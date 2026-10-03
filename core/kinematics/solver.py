"""JAX-backed kinematic assembly session.

One ``KinematicSolver`` is one session: prepared arrays, workspace, and the
compiled evaluator. Dragging and Solve Assembly both use this session. The
iteration loop is the shared NumPy engine. Constraint evaluation is JAX.

``solve_assembly`` and ``solve_drag`` read the attached ``State`` and, when
that state exists, commit finite poses back into it. That keeps headless tests
on the historical call shape. The GUI worker uses ``solve_request`` instead,
which reads an owned pose snapshot and does not write the live document.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, replace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from core.data_structures import Joint, State
from core.kinematics.backends import get_evaluator
from core.kinematics.engine import (
    diagnose_model,
    poses_finite,
    snapshot_state,
    solve_assembly as _solve_assembly,
    solve_drag as _solve_drag,
)
from core.kinematics.linear_sparse import make_linear_strategy
from core.kinematics.prepared import (
    FORMULATION_COMPAT,
    GROUPING_COMPATIBILITY,
    constant_signature,
    prepare,
    refresh_constants,
    structural_signature,
)
from core.kinematics.reports import RevisionStamp, SolveReport, apply_stamp, commit_poses
from core.kinematics.trace import PhaseTrace, SolveOptions
from core.kinematics.workspace import ensure_workspace, validate_workspace


@dataclass
class SolveRequest:
    """Owned numerical inputs for one worker request."""

    kind: str
    bodies: list
    joints: list
    poses: Dict[int, Tuple[np.ndarray, np.ndarray]]
    ground_pose: Tuple[np.ndarray, np.ndarray]
    stamp: RevisionStamp
    dragged_body_id: Optional[int] = None
    target_origin: Optional[np.ndarray] = None
    target_rotation: Optional[np.ndarray] = None
    max_iters: int = 30
    tol: float = 1e-8
    pin_weight: float = 1.0
    pin_orientation: bool = False
    analyze: bool = False
    locked_body_ids: Tuple[int, ...] = ()


class KinematicSolver:
    """Persistent JAX session over the shared Levenberg-Marquardt engine."""

    def __init__(
        self,
        bodies,
        joints: Sequence[Joint],
        state: Optional[State] = None,
        ground_id: int = -1,
        ground_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        locked_body_ids: Optional[Iterable[int]] = None,
        formulation_version: str = FORMULATION_COMPAT,
        grouping: str = GROUPING_COMPATIBILITY,
        diagnostics: str = "policy",
        linear_solver: str = "dense",
    ):
        if diagnostics not in ("policy", "compat", "off"):
            raise ValueError("diagnostics must be 'policy', 'compat', or 'off'.")
        self.bodies = bodies if isinstance(bodies, list) else list(bodies)
        self.joints = joints if isinstance(joints, list) else list(joints)
        self.state = state
        self.ground_id = int(ground_id)
        self.ground_pose = ground_pose if ground_pose is not None else (np.zeros(3), np.eye(3))
        self.locked_body_ids = set(int(b) for b in (locked_body_ids or []))
        self.locked_body_ids.add(self.ground_id)
        self.formulation_version = formulation_version
        self.grouping = grouping
        self.diagnostics = diagnostics
        self.linear_solver_name = linear_solver
        self.linear_strategy = make_linear_strategy(linear_solver)
        self._model = None
        self._workspace = None
        self._topology_revision = None
        self._marker_revision = None
        self.evaluator = get_evaluator("jax")

    def _ensure_workspace(self, model):
        self._workspace = ensure_workspace(
            self._workspace, model,
            dense_linear=not getattr(self.linear_strategy, "uses_blocks", False),
        )
        return self._workspace

    def release(self) -> None:
        """Drop this session's prepared data and compiled-function references."""
        self._model = None
        self._workspace = None
        self._topology_revision = None
        self._marker_revision = None
        release = getattr(self.evaluator, "release", None)
        if release is not None:
            release()

    def invalidate(self, category: str) -> None:
        """Drop cached preparation for topology, marker, or pose changes."""
        key = category.strip().lower()
        if key in ("topology", "structure", "locks", "bodies", "joints"):
            self._model = None
            self._workspace = None
            self._topology_revision = None
            self._marker_revision = None
        elif key in ("constants", "markers", "axes"):
            if self._model is not None:
                self._model = replace(self._model, constant_signature="invalid")
            self._marker_revision = None
        elif key in ("poses", "state"):
            pass
        else:
            raise ValueError(
                f"Unknown invalidation category {category!r}. "
                "Use 'topology', 'constants', or 'poses'."
            )

    def prewarm(self) -> None:
        """Compile both evaluator kernels before a timed drag."""
        trace = PhaseTrace(enabled=False)
        model = self._ensure_model(trace, None)
        self._ensure_workspace(model)
        self._load_start(model, self._workspace, None)
        self.evaluator.prewarm(model, self._workspace.origin, self._workspace.rotation)

    def solve_assembly(
        self,
        max_iters: int = 50,
        tol: float = 1e-9,
        analyze: Optional[bool] = None,
        trace: bool = False,
        stamp: Optional[RevisionStamp] = None,
        commit: Optional[bool] = None,
    ) -> SolveReport:
        do_analyze = self._wants_analysis(analyze, drag=False)
        options = SolveOptions(
            max_iters=max_iters,
            tol=tol,
            pin_weight=0.0,
            pin_orientation=True,
            joint_weight=1e3,
            analyze=do_analyze,
            trace=trace,
            strategy=self.linear_strategy,
        )
        return self._run(options, drag=None, stamp=stamp, commit=commit, pose_map=None)

    def solve_drag(
        self,
        dragged_body_id: int,
        target_origin: np.ndarray,
        target_R: Optional[np.ndarray] = None,
        pin_weight: float = 100.0,
        max_iters: int = 30,
        tol: float = 1e-8,
        pin_orientation: bool = False,
        trace: bool = False,
        analyze: Optional[bool] = None,
        stamp: Optional[RevisionStamp] = None,
        commit: Optional[bool] = None,
    ) -> SolveReport:
        if target_R is None:
            target_R = self._current_rotation(int(dragged_body_id))
        do_analyze = self._wants_analysis(analyze, drag=True)
        options = SolveOptions(
            max_iters=max_iters,
            tol=tol,
            pin_weight=pin_weight,
            pin_orientation=pin_orientation,
            joint_weight=1e3,
            analyze=do_analyze,
            trace=trace,
            strategy=self.linear_strategy,
        )
        drag = (
            int(dragged_body_id),
            np.asarray(target_origin, dtype=np.float64),
            np.asarray(target_R, dtype=np.float64),
        )
        return self._run(options, drag=drag, stamp=stamp, commit=commit, pose_map=None)

    def solve_request(self, request: SolveRequest) -> SolveReport:
        """Solve from an owned snapshot. Does not write a live ``State``."""
        self.bodies = list(request.bodies)
        self.joints = list(request.joints)
        self.ground_pose = (
            np.array(request.ground_pose[0], dtype=np.float64, copy=True),
            np.array(request.ground_pose[1], dtype=np.float64, copy=True),
        )
        self.locked_body_ids = set(int(b) for b in request.locked_body_ids)
        self.locked_body_ids.add(self.ground_id)
        kind = request.kind
        if kind == "prewarm":
            trace = PhaseTrace(enabled=False)
            try:
                model = self._ensure_model(trace, request.stamp)
                self._ensure_workspace(model)
                self._load_start(model, self._workspace, request.poses)
                self.evaluator.prewarm(model, self._workspace.origin, self._workspace.rotation)
            except ValueError as exc:
                return self._invalid(f"Invalid model: {exc}", request.stamp, trace)
            report = SolveReport(True, 0, 0.0, 0.0, message="Prepared", finite=True, trace=trace)
            apply_stamp(report, request.stamp)
            self._copy_compile_stats(trace)
            return report
        if kind == "diagnose":
            return self._diagnose(request)
        if kind == "assembly":
            return self._run(
                SolveOptions(
                    max_iters=request.max_iters,
                    tol=request.tol,
                    pin_weight=0.0,
                    pin_orientation=True,
                    joint_weight=1e3,
                    analyze=True,
                    trace=False,
                    strategy=self.linear_strategy,
                ),
                drag=None,
                stamp=request.stamp,
                commit=False,
                pose_map=request.poses,
            )
        analyze = bool(request.analyze)
        return self._run(
            SolveOptions(
                max_iters=request.max_iters,
                tol=request.tol,
                pin_weight=request.pin_weight,
                pin_orientation=request.pin_orientation,
                joint_weight=1e3,
                analyze=analyze,
                trace=False,
                strategy=self.linear_strategy,
            ),
            drag=(
                int(request.dragged_body_id),
                np.asarray(request.target_origin, dtype=np.float64),
                np.asarray(request.target_rotation, dtype=np.float64),
            ),
            stamp=request.stamp,
            commit=False,
            pose_map=request.poses,
        )

    def _wants_analysis(self, override: Optional[bool], drag: bool) -> bool:
        if override is not None:
            return bool(override)
        if self.diagnostics == "off":
            return False
        # Interactive dragging skips rank analysis. Assembly and an explicit
        # final diagnostic request turn it back on.
        if drag:
            return False
        return True

    def _current_rotation(self, body_id: int) -> np.ndarray:
        if self.state is not None:
            pose = self.state.get_body_pose(body_id)
            if pose is not None:
                return np.array(pose.rotation_matrix, dtype=np.float64, copy=True)
        return np.eye(3)

    def _ground_pose_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        origin, rotation = self.ground_pose
        return (
            np.asarray(origin, dtype=np.float64),
            np.asarray(rotation, dtype=np.float64),
        )

    def _ensure_model(self, trace: PhaseTrace, stamp: Optional[RevisionStamp]):
        use_revisions = stamp is not None and stamp.use_revisions
        if use_revisions:
            if self._model is None or stamp.topology_revision != self._topology_revision:
                trace.stale_model = self._model is not None
                self._model = self._prepare()
                self._workspace = None
                self._topology_revision = stamp.topology_revision
                self._marker_revision = stamp.marker_revision
            elif stamp.marker_revision != self._marker_revision:
                trace.stale_model = True
                self._model = refresh_constants(self._model, self.joints)
                self._marker_revision = stamp.marker_revision
            self._assert_revisions(stamp)
            return self._model

        struct = structural_signature(
            [int(b.id) for b in self.bodies],
            self.joints,
            self.locked_body_ids,
            self.ground_id,
            self.formulation_version,
            self.grouping,
        )
        const = constant_signature(
            [int(b.id) for b in self.bodies],
            self.joints,
            self.locked_body_ids,
            self.ground_id,
            self.formulation_version,
            self.grouping,
        )
        if self._model is None or struct != self._model.structural_signature:
            if self._model is not None:
                trace.stale_model = True
            self._model = self._prepare()
            self._workspace = None
        elif const != self._model.constant_signature:
            trace.stale_model = True
            self._model = refresh_constants(self._model, self.joints)
        return self._model

    def _prepare(self):
        return prepare(
            self.bodies,
            self.joints,
            self.locked_body_ids,
            ground_id=self.ground_id,
            formulation_version=self.formulation_version,
            grouping=self.grouping,
            revision=1,
        )

    def _assert_revisions(self, stamp: RevisionStamp) -> None:
        if os.environ.get("MBD_ASSERT_SIGNATURES") != "1" or self._model is None:
            return
        struct = structural_signature(
            [int(b.id) for b in self.bodies],
            self.joints,
            self.locked_body_ids,
            self.ground_id,
            self.formulation_version,
            self.grouping,
        )
        if struct != self._model.structural_signature:
            raise AssertionError(
                "Topology changed without a topology revision. "
                f"request={stamp.topology_revision}"
            )

    def _load_start(self, model, workspace, pose_map) -> None:
        if pose_map is None and self.state is not None:
            snapshot_state(self.state, model, workspace, self._ground_pose_arrays())
            return
        ground_origin, ground_rotation = self._ground_pose_arrays()
        workspace.origin[0] = ground_origin
        workspace.rotation[0] = ground_rotation
        eye = np.eye(3)
        poses = pose_map or {}
        for body_id in model.body_ids:
            slot = model.slot_of[int(body_id)]
            pair = poses.get(int(body_id))
            if pair is None and self.state is not None:
                pose = self.state.get_body_pose(int(body_id))
                if pose is not None:
                    pair = (pose.origin, pose.rotation_matrix)
            if pair is None:
                workspace.origin[slot] = 0.0
                workspace.rotation[slot] = eye
            else:
                workspace.origin[slot] = np.asarray(pair[0], dtype=np.float64)
                workspace.rotation[slot] = np.asarray(pair[1], dtype=np.float64)
        np.copyto(workspace.trial_origin, workspace.origin)
        np.copyto(workspace.trial_rotation, workspace.rotation)

    def _copy_poses(self, body_ids: Sequence[int]) -> Dict[int, Tuple[np.ndarray, np.ndarray]]:
        poses = {}
        for body_id in body_ids:
            body_id = int(body_id)
            slot = self._model.slot_of[body_id]
            poses[body_id] = (
                np.array(self._workspace.origin[slot], dtype=np.float64, copy=True),
                np.array(self._workspace.rotation[slot], dtype=np.float64, copy=True),
            )
        return poses

    def _copy_compile_stats(self, trace: PhaseTrace) -> None:
        trace.compile_count = int(getattr(self.evaluator, "compile_count", 0))
        trace.compile_s = float(getattr(self.evaluator, "compile_s", 0.0))
        trace.backend = getattr(self.evaluator, "name", "jax")
        trace.device = getattr(self.evaluator, "device", "cpu")

    def _finish(
        self,
        report: SolveReport,
        trace: PhaseTrace,
        moved: Sequence[int],
        finite: bool,
        stamp: Optional[RevisionStamp],
        commit: Optional[bool],
        materialize_before: float,
    ) -> SolveReport:
        report.finite = bool(finite)
        report.trace = trace
        report.joint_feasible = bool(trace.joint_feasible) if finite else False
        report.target_error = trace.pin_error
        apply_stamp(report, stamp)
        self._copy_compile_stats(trace)
        if trace.enabled:
            trace.add_time(
                "materialize",
                float(getattr(self.evaluator, "materialize_s", 0.0)) - materialize_before,
            )
        if not finite or self._model is None or self._workspace is None:
            report.poses = {}
            report.finite = False
            report.converged = False
            if not report.message:
                report.message = "Numerical failure"
            return report
        if not poses_finite(self._workspace, moved, self._model):
            report.poses = {}
            report.finite = False
            report.converged = False
            report.message = "Numerical failure"
            trace.numerical_failure = True
            return report
        report.poses = self._copy_poses(moved)
        report.moved_bodies = list(report.poses.keys())
        do_commit = (self.state is not None) if commit is None else bool(commit)
        if do_commit and self.state is not None:
            commit_poses(self.state, report)
        return report

    def _invalid(self, message: str, stamp: Optional[RevisionStamp], trace: PhaseTrace) -> SolveReport:
        report = SolveReport(
            converged=False,
            iterations=0,
            final_residual_norm=float("inf"),
            max_residual=float("inf"),
            message=message,
            finite=False,
            trace=trace,
        )
        apply_stamp(report, stamp)
        self._copy_compile_stats(trace)
        return report

    def _run(
        self,
        options: SolveOptions,
        drag: Optional[tuple],
        stamp: Optional[RevisionStamp],
        commit: Optional[bool],
        pose_map,
    ) -> SolveReport:
        trace = PhaseTrace(enabled=options.trace)
        materialize_before = float(getattr(self.evaluator, "materialize_s", 0.0))
        try:
            with trace.phase("prepare"):
                model = self._ensure_model(trace, stamp)
                self._ensure_workspace(model)
                validate_workspace(self._workspace)
        except ValueError as exc:
            return self._invalid(f"Invalid model: {exc}", stamp, trace)
        with trace.phase("snapshot"):
            self._load_start(model, self._workspace, pose_map)
        if drag is None:
            result = _solve_assembly(self.evaluator, model, self._workspace, options, trace)
        else:
            body_id, target_origin, target_rotation = drag
            result = _solve_drag(
                self.evaluator, model, self._workspace, options,
                body_id, target_origin, target_rotation, trace,
            )
        return self._finish(
            result.report, result.trace, result.moved_body_ids, result.finite,
            stamp, commit, materialize_before,
        )

    def _diagnose(self, request: SolveRequest) -> SolveReport:
        trace = PhaseTrace(enabled=False)
        materialize_before = float(getattr(self.evaluator, "materialize_s", 0.0))
        try:
            with trace.phase("prepare"):
                model = self._ensure_model(trace, request.stamp)
                self._ensure_workspace(model)
                validate_workspace(self._workspace)
        except ValueError as exc:
            return self._invalid(f"Invalid model: {exc}", request.stamp, trace)
        with trace.phase("snapshot"):
            self._load_start(model, self._workspace, request.poses)
        options = SolveOptions(
            max_iters=0,
            tol=request.tol,
            analyze=True,
            trace=False,
            strategy=self.linear_strategy,
        )
        result = diagnose_model(self.evaluator, model, self._workspace, options, trace)
        report = self._finish(
            result.report, result.trace, [], result.finite,
            request.stamp, False, materialize_before,
        )
        # Diagnostics describe the pose they were given. They do not move it.
        report.poses = {}
        report.moved_bodies = []
        report.kind = "diagnose"
        return report
