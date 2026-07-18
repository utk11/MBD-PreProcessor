"""Damped least-squares kinematic assembly solver.

Solves body 6DOF poses so that all joint constraints are satisfied, using a
Levenberg-Marquardt style damped Newton loop over stacked per-joint residuals
with analytic Jacobians (see core/kinematics/constraints.py).

Design (the "SolveSpace copy"):
  * Full-Cartesian body-6DOF coordinates; ground contributes no unknowns.
  * Optional *pinning* of a dragged body: the dragged body's pose is a soft,
    high-weight constraint (its desired pose), so the solver balances the
    user's intent against the joint constraints.
  * Group decomposition via JointGraph: only the connected component(s) that
    need solving are solved.
  * After solving, per-joint residual norms, an estimate of redundant
    constraints, and the mechanism mobility (DOF) are reported from the
    assembled Jacobian.

Poses are read from / written to a ``State`` object, so the existing renderer
(``update_body_transform``) picks up solved poses unchanged.

Pure numpy; no GUI / OCC imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from core.data_structures import Joint, Pose, State
from core.kinematics import markers as M
from core.kinematics.constraints import JointConstraint, CONSTRAINT_COUNT
from core.kinematics.graph import JointGraph


@dataclass
class SolveReport:
    """Outcome of a solve."""
    converged: bool
    iterations: int
    final_residual_norm: float
    max_residual: float
    per_joint_residual: Dict[str, float] = field(default_factory=dict)
    moved_bodies: List[int] = field(default_factory=list)
    redundant_joints: List[str] = field(default_factory=list)
    dof: Optional[int] = None
    message: str = ""


class KinematicSolver:
    """Position-level assembly solver over a body/joint graph."""

    def __init__(self,
                 bodies,                      # iterable of RigidBody (need .id)
                 joints: List[Joint],
                 state: State,
                 ground_id: int = -1,
                 ground_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                 locked_body_ids: Optional[List[int]] = None):
        self.state = state
        self.joints = list(joints)
        self.ground_id = ground_id
        self.ground_pose = ground_pose if ground_pose is not None else (
            np.zeros(3), np.eye(3))
        self.locked_body_ids = set(locked_body_ids or [])
        self.locked_body_ids.add(ground_id)

        self.body_ids = [b.id for b in bodies]
        self.graph = JointGraph(self.body_ids, self.joints, ground_id=ground_id)

    # ------------------------------------------------------------------
    # Pose access
    # ------------------------------------------------------------------
    def _get_pose(self, body_id: int) -> Tuple[np.ndarray, np.ndarray]:
        pose = self.state.get_body_pose(body_id)
        if pose is None:
            return np.zeros(3), np.eye(3)
        return pose.origin.copy(), pose.rotation_matrix.copy()

    def _set_pose(self, body_id: int, origin: np.ndarray, R: np.ndarray):
        self.state.set_body_pose(body_id, np.asarray(origin, float),
                                 M.project_to_so3(np.asarray(R, float)))

    # ------------------------------------------------------------------
    # Assembly of the nonlinear system for a set of joints / movable bodies
    # ------------------------------------------------------------------
    def _build(self, joints: List[Joint], movable: List[int]):
        """Return (constraints, residual_fn, jacobian_fn, col_index)."""
        col_index: Dict[int, int] = {bid: 6 * i for i, bid in enumerate(movable)}
        ncols = 6 * len(movable)

        constraints = [
            JointConstraint(j, self._get_pose,
                            ground_id=self.ground_id,
                            ground_pose=self.ground_pose)
            for j in joints
        ]

        def residual_fn() -> np.ndarray:
            return np.concatenate([c.residual() for c in constraints]) \
                if constraints else np.zeros(0)

        def jacobian_fn() -> np.ndarray:
            if not constraints:
                return np.zeros((0, ncols))
            rows = []
            for c in constraints:
                Jc, mov = c.jacobian()
                Jr = np.zeros((Jc.shape[0], ncols))
                for k, bid in enumerate(mov):
                    if bid in col_index:
                        Jr[:, col_index[bid]:col_index[bid] + 6] = Jc[:, 6 * k:6 * k + 6]
                rows.append(Jr)
            return np.vstack(rows) if rows else np.zeros((0, ncols))

        return constraints, residual_fn, jacobian_fn, col_index

    # ------------------------------------------------------------------
    # LM solve on one component
    # ------------------------------------------------------------------
    def _solve_lm(self,
                  joints: List[Joint],
                  movable: List[int],
                  pin_body_id: Optional[int],
                  pin_target: Optional[Tuple[np.ndarray, np.ndarray]],
                  pin_weight: float,
                  max_iters: int,
                  tol: float,
                  pin_orientation: bool = True,
                  joint_weight: float = 1e3) -> SolveReport:
        constraints, residual_fn, jacobian_fn, col_index = self._build(joints, movable)

        pin_rows = 0
        if pin_body_id is not None and pin_body_id in col_index and pin_target is not None:
            pin_rows = 6 if pin_orientation else 3

        lam = 1e-3
        report = SolveReport(converged=False, iterations=0,
                             final_residual_norm=np.inf, max_residual=np.inf)

        def full_residual():
            # Joints are "hard" (high weight); pin is soft.
            r = residual_fn() * joint_weight
            if pin_rows and pin_target is not None and pin_body_id is not None:
                o_t, R_t = pin_target
                o_c, R_c = self._get_pose(pin_body_id)
                sw = np.sqrt(pin_weight)
                if pin_orientation:
                    r_pin = np.concatenate([
                        o_c - o_t,
                        M.relative_rotation_vector(R_c, R_t),
                    ]) * sw
                else:
                    r_pin = (o_c - o_t) * sw
                r = np.concatenate([r, r_pin])
            return r

        def full_jacobian():
            J = jacobian_fn() * joint_weight
            if pin_rows and pin_target is not None and pin_body_id is not None:
                sw = np.sqrt(pin_weight)
                c0 = col_index[pin_body_id]
                if pin_orientation:
                    J_pin = np.zeros((6, J.shape[1]))
                    J_pin[0:3, c0:c0 + 3] = np.eye(3) * sw
                    J_pin[3:6, c0 + 3:c0 + 6] = -np.eye(3) * sw
                else:
                    J_pin = np.zeros((3, J.shape[1]))
                    J_pin[0:3, c0:c0 + 3] = np.eye(3) * sw
                J = np.vstack([J, J_pin])
            return J

        for it in range(max_iters):
            r = full_residual()
            J = full_jacobian()

            r_joints = residual_fn()
            jnorm = float(np.linalg.norm(r_joints)) if r_joints.size else 0.0
            jmax = float(np.max(np.abs(r_joints))) if r_joints.size else 0.0
            report.iterations = it + 1
            report.final_residual_norm = jnorm
            report.max_residual = jmax

            if J.shape[1] == 0:
                report.converged = (jmax < tol)
                break

            # Damped normal equations: (J^T J + lam * diag(J^T J)) d = -J^T r
            JTJ = J.T @ J
            g = J.T @ r
            diag = np.diag(JTJ).copy()
            diag[diag < 1e-12] = 1e-12
            try:
                delta = np.linalg.solve(JTJ + lam * np.diag(diag), -g)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(JTJ + lam * np.diag(diag), -g,
                                        rcond=None)[0]

            step_norm = float(np.linalg.norm(delta))
            # Stop when joints are satisfied and the step is tiny.
            # (With an active pin the joints may already be zero at the start of
            # a drag, so we must not exit before taking a pin-driven step.)
            if jmax < tol and step_norm < max(tol, 1e-10):
                report.converged = True
                break

            # Trial apply.
            old_poses = {bid: self._get_pose(bid) for bid in movable}
            for bid in movable:
                c0 = col_index[bid]
                o, R = old_poses[bid]
                no, nR = M.apply_increment(o, R, delta[c0:c0 + 6])
                self._set_pose(bid, no, nR)

            new_norm = float(np.linalg.norm(full_residual()))
            old_norm = float(np.linalg.norm(r))
            if new_norm < old_norm * (1.0 + 1e-8):
                lam = max(lam * 0.5, 1e-9)
            else:
                for bid in movable:
                    o, R = old_poses[bid]
                    self._set_pose(bid, o, R)
                lam = min(lam * 4.0, 1e6)

        # Final residual snapshot (joint constraints only — not pin).
        r_final = residual_fn()
        report.final_residual_norm = float(np.linalg.norm(r_final)) if r_final.size else 0.0
        report.max_residual = float(np.max(np.abs(r_final))) if r_final.size else 0.0
        report.converged = report.converged or (report.max_residual < max(tol * 10, 1e-6))

        for c in constraints:
            rc = c.residual()
            report.per_joint_residual[c.joint.name] = float(np.linalg.norm(rc))

        report.moved_bodies = list(movable)
        return report

    # ------------------------------------------------------------------
    # Redundancy / DOF analysis on the assembled Jacobian
    # ------------------------------------------------------------------
    def _analyze(self, joints: List[Joint], movable: List[int],
                 tol: float = 1e-8) -> Tuple[List[str], Optional[int]]:
        if not movable:
            return [], 0
        _, residual_fn, jacobian_fn, _ = self._build(joints, movable)
        J = jacobian_fn()
        if J.size == 0:
            return [], 6 * len(movable)
        sv = np.linalg.svd(J, compute_uv=False)
        rank = int(np.sum(sv > max(tol, sv[0] * 1e-8 if sv.size else 0.0)))
        n_eq = J.shape[0]
        n_unknown = J.shape[1]

        # Redundant joints: a joint is flagged if removing its rows does NOT
        # decrease the Jacobian rank (its rows were linearly dependent on the
        # rest).  Trigger whenever we have more equations than independent ones.
        redundant: List[str] = []
        if n_eq > rank:
            names = [j.name for j in joints]
            counts = [CONSTRAINT_COUNT[j.joint_type] for j in joints]
            # Row-slice per joint and test rank drop on removal.
            start = 0
            for name, cnt in zip(names, counts):
                idx = list(range(0, start)) + list(range(start + cnt, n_eq))
                if idx:
                    Jred = J[idx, :]
                    sv2 = np.linalg.svd(Jred, compute_uv=False)
                    thr = max(tol, (sv2[0] * 1e-8 if sv2.size else 0.0))
                    rank2 = int(np.sum(sv2 > thr))
                else:
                    rank2 = 0
                if rank2 == rank:
                    redundant.append(name)
                start += cnt

        dof = max(0, n_unknown - rank)
        return redundant, dof

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------
    def solve_assembly(self,
                       max_iters: int = 50,
                       tol: float = 1e-9,
                       analyze: bool = True) -> SolveReport:
        """Solve all components to satisfy all joints (the 'snap' command)."""
        movable_all = [b for b in self.body_ids if b not in self.locked_body_ids]
        if not self.joints:
            return SolveReport(True, 0, 0.0, 0.0, message="No joints to solve.")

        # Solve each connected component independently.
        overall = SolveReport(True, 0, 0.0, 0.0)
        seen: set = set()
        for comp in self.graph.components():
            comp_bodies = [b for b in comp
                           if b in self.body_ids and b not in self.locked_body_ids]
            comp_joints = [j for j in self.joints
                           if j.body1_id in comp and j.body2_id in comp]
            if not comp_joints:
                continue
            key = tuple(sorted(comp))
            if key in seen:
                continue
            seen.add(key)

            rep = self._solve_lm(comp_joints, comp_bodies,
                                 pin_body_id=None, pin_target=None,
                                 pin_weight=0.0,
                                 max_iters=max_iters, tol=tol,
                                 pin_orientation=True)
            overall.iterations += rep.iterations
            overall.converged = overall.converged and rep.converged
            overall.per_joint_residual.update(rep.per_joint_residual)
            overall.moved_bodies.extend(rep.moved_bodies)
            overall.final_residual_norm = max(overall.final_residual_norm,
                                              rep.final_residual_norm)
            overall.max_residual = max(overall.max_residual, rep.max_residual)

        if analyze:
            red, dof = self._analyze(self.joints, movable_all)
            overall.redundant_joints = red
            overall.dof = dof

        overall.message = ("Converged" if overall.converged else "Did not fully converge")
        return overall

    def solve_drag(self,
                   dragged_body_id: int,
                   target_origin: np.ndarray,
                   target_R: Optional[np.ndarray] = None,
                   pin_weight: float = 100.0,
                   max_iters: int = 30,
                   tol: float = 1e-8,
                   pin_orientation: bool = False) -> SolveReport:
        """Solve the dragged body's component with the dragged body pinned to a target.

        Only the connected component containing ``dragged_body_id`` is solved;
        unrelated bodies are untouched.

        By default only the *position* is pinned (``pin_orientation=False``),
        matching mouse-drag translation. Pass ``pin_orientation=True`` to also
        soft-constrain orientation.
        """
        comp = self.graph.component_of(dragged_body_id)
        comp_joints = [j for j in self.joints
                       if j.body1_id in comp and j.body2_id in comp]
        movable = [b for b in comp
                   if b in self.body_ids and b not in self.locked_body_ids]

        if not comp_joints or not movable:
            return SolveReport(True, 0, 0.0, 0.0,
                               message="Dragged body is not joint-constrained.")

        if target_R is None:
            _, target_R = self._get_pose(dragged_body_id)

        rep = self._solve_lm(comp_joints, movable,
                             pin_body_id=dragged_body_id,
                             pin_target=(np.asarray(target_origin, float),
                                         np.asarray(target_R, float)),
                             pin_weight=pin_weight,
                             max_iters=max_iters, tol=tol,
                             pin_orientation=pin_orientation)

        red, dof = self._analyze(comp_joints, movable)
        rep.redundant_joints = red
        rep.dof = dof
        rep.message = "Converged" if rep.converged else "Did not fully converge"
        return rep
