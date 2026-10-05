"""Shared Levenberg-Marquardt engine for the packed evaluators.

Assembly iteration, weights, damping, acceptance, and the dense linear step
match ``KinematicSolver._solve_lm`` in formulation ``compat-v1``. Drag trials
use the same joint equations with a predictor and constraint correction.
Trial poses stay in the workspace; this module does not write the live ``State``.

Assembly stopping behavior keeps the compatibility trace of the legacy
solver. Dragging uses pin-scaled initial damping and corrects trial poses
back onto the joint constraints before accepting mouse-driven motion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from core.data_structures import State
import core.kinematics.markers as M
from core.kinematics.backends import ConstraintEvaluator, assemble_blocks
from core.kinematics.diagnostics import analyze_jacobian
from core.kinematics.linear import solve_dense_damped
from core.kinematics.prepared import ComponentPlan, PreparedModel
from core.kinematics.reports import SolveReport
from core.kinematics.trace import PhaseTrace, SolveOptions
from core.kinematics.workspace import Workspace


# Legacy constants from KinematicSolver._solve_lm.
_LAM0 = 1e-3
_LAM_MIN = 1e-9
_LAM_MAX = 1e6
_ACCEPT_REL = 1e-8
_DRAG_MAX_ROTATION = 0.1  # Radians per trial; preserve the local linkage branch.


def _record_linear_work(trace: PhaseTrace, step) -> None:
    """Count primary and drag-correction solves, including failed iterations."""
    trace.linear_solves += 1
    trace.linear_iterations += int(step.iterations or 0)
    trace.linear_failures += int(not step.finite)


@dataclass
class EngineResult:
    report: SolveReport
    trace: PhaseTrace
    moved_body_ids: List[int] = field(default_factory=list)
    finite: bool = True


def snapshot_state(
    state: State,
    model: PreparedModel,
    workspace: Workspace,
    ground_pose: Tuple[np.ndarray, np.ndarray],
) -> None:
    """Copy public poses into the private buffers. Ground is slot 0."""
    ground_origin, ground_rotation = ground_pose
    workspace.origin[0] = np.asarray(ground_origin, dtype=np.float64)
    workspace.rotation[0] = np.asarray(ground_rotation, dtype=np.float64)
    eye = np.eye(3)
    for body_id in model.body_ids:
        slot = model.slot_of[int(body_id)]
        pose = state.get_body_pose(int(body_id))
        if pose is None:
            workspace.origin[slot] = 0.0
            workspace.rotation[slot] = eye
        else:
            workspace.origin[slot] = pose.origin
            workspace.rotation[slot] = pose.rotation_matrix
    np.copyto(workspace.trial_origin, workspace.origin)
    np.copyto(workspace.trial_rotation, workspace.rotation)


def poses_finite(workspace: Workspace, body_ids: Sequence[int], model: PreparedModel) -> bool:
    for body_id in body_ids:
        slot = model.slot_of[int(body_id)]
        if not np.isfinite(workspace.origin[slot]).all():
            return False
        if not np.isfinite(workspace.rotation[slot]).all():
            return False
    return True


def _weighted_norm(joint_residual: np.ndarray, joint_weight: float, pin: Optional[np.ndarray]) -> float:
    total = float(joint_weight * joint_weight * np.dot(joint_residual, joint_residual))
    if pin is not None and pin.size:
        total += float(np.dot(pin, pin))
    if total < 0.0 or not np.isfinite(total):
        return float("inf")
    return float(np.sqrt(total))


def _write_pin(
    out: np.ndarray,
    origin_row: np.ndarray,
    rotation_row: np.ndarray,
    target_origin: np.ndarray,
    target_rotation: np.ndarray,
    orientation: bool,
    weight_sqrt: float,
) -> int:
    out[0:3] = (origin_row - target_origin) * weight_sqrt
    if not orientation:
        return 3
    out[3:6] = M.log_so3(target_rotation @ rotation_row.T) * weight_sqrt
    return 6


def _write_pin_jacobian(
    jacobian: np.ndarray,
    row0: int,
    col0: int,
    orientation: bool,
    weight_sqrt: float,
) -> None:
    width = 6 if orientation else 3
    jacobian[row0:row0 + width, :] = 0.0
    jacobian[row0:row0 + 3, col0:col0 + 3] = weight_sqrt * np.eye(3)
    if orientation:
        jacobian[row0 + 3:row0 + 6, col0 + 3:col0 + 6] = -weight_sqrt * np.eye(3)


def _apply_trial(
    workspace: Workspace,
    slots: Sequence[int],
    delta: np.ndarray,
) -> None:
    np.copyto(workspace.trial_origin, workspace.origin)
    np.copyto(workspace.trial_rotation, workspace.rotation)
    for index, slot in enumerate(slots):
        column = 6 * index
        updated_origin, updated_rotation = M.apply_increment(
            workspace.origin[slot], workspace.rotation[slot], delta[column:column + 6]
        )
        workspace.trial_origin[slot] = updated_origin
        workspace.trial_rotation[slot] = M.project_to_so3(updated_rotation)


def _accept_trial(workspace: Workspace, slots: Sequence[int]) -> None:
    for slot in slots:
        workspace.origin[slot] = workspace.trial_origin[slot]
        workspace.rotation[slot] = workspace.trial_rotation[slot]


def _project_current(workspace: Workspace, slots: Sequence[int]) -> None:
    """Match the legacy rejected-step restore, which re-projects the saved pose."""
    for slot in slots:
        workspace.rotation[slot] = M.project_to_so3(workspace.rotation[slot])


def _correct_drag_trial(evaluator, model, workspace, plan, options, trace):
    """Retract a mouse-driven trial onto the curved joint constraint surface.

    A tangent step satisfies joints only to first order. Without correction,
    their large weights reject useful rotations and dragging barely advances.
    Joint-only damped corrections preserve the free motion while closing joints.
    Accepted poses stay untouched until the corrected trial is evaluated.
    """
    nrows, ncols = plan.n_residual_rows, plan.ncols
    if not nrows or not ncols:
        return
    strategy = getattr(options, "strategy", None)
    use_blocks = strategy is not None and getattr(strategy, "uses_blocks", False)
    for _ in range(5):
        evaluator.evaluate(
            model, workspace.trial_origin, workspace.trial_rotation, plan.joint_indices,
            workspace.residual, workspace.blocks, True,
        )
        residual = workspace.residual[:nrows]
        if not np.isfinite(residual).all() or np.max(np.abs(residual)) <= min(options.tol * 0.1, 1e-8):
            return
        workspace.weighted[:nrows] = residual
        if use_blocks:
            step = strategy.solve_from_blocks(
                plan, model, workspace.blocks, workspace.weighted[:nrows], _LAM_MIN,
                joint_weight=1.0, n_joint_rows=nrows, pin_column=None,
                pin_width=0, pin_sqrt=0.0, delta_out=workspace.delta,
            )
        else:
            assemble_blocks(plan, model, workspace.blocks, workspace.jacobian)
            step = solve_dense_damped(
                workspace.jacobian[:nrows, :ncols], workspace.weighted[:nrows], _LAM_MIN,
                workspace.normal, workspace.gradient, workspace.delta, workspace.system,
            )
        _record_linear_work(trace, step)
        if not step.finite:
            return
        for index, slot in enumerate(plan.movable_slots):
            column = 6 * index
            origin, rotation = M.apply_increment(
                workspace.trial_origin[slot], workspace.trial_rotation[slot],
                workspace.delta[column:column + 6],
            )
            workspace.trial_origin[slot] = origin
            workspace.trial_rotation[slot] = M.project_to_so3(rotation)


def _joint_norms(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    plan: ComponentPlan,
) -> Dict[str, float]:
    if plan.n_residual_rows == 0:
        return {name: 0.0 for name in plan.joint_names}
    evaluator.evaluate(
        model,
        workspace.origin,
        workspace.rotation,
        plan.joint_indices,
        workspace.residual,
        workspace.blocks,
        False,
    )
    norms: Dict[str, float] = {}
    cursor = 0
    for local, name in enumerate(plan.joint_names):
        width = int(plan.row_counts[local])
        norms[name] = float(np.linalg.norm(workspace.residual[cursor:cursor + width]))
        cursor += width
    return norms


def _solve_plan(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    plan: ComponentPlan,
    options: SolveOptions,
    trace: PhaseTrace,
    pin_body_id: Optional[int],
    pin_target: Optional[Tuple[np.ndarray, np.ndarray]],
) -> Tuple[SolveReport, bool]:
    """Iterate one component. Returns the report and whether poses stayed finite."""
    n_joint = plan.n_residual_rows
    ncols = plan.ncols
    slots = plan.movable_slots.tolist()
    pin_col = None
    pin_slot = None
    target_origin = None
    target_rotation = None
    if (
        pin_body_id is not None
        and pin_target is not None
        and pin_body_id in plan.movable_body_ids
    ):
        pin_index = plan.movable_body_ids.index(int(pin_body_id))
        pin_col = 6 * pin_index
        pin_slot = int(plan.movable_slots[pin_index])
        target_origin = np.asarray(pin_target[0], dtype=np.float64)
        target_rotation = np.asarray(pin_target[1], dtype=np.float64)
    pin_sqrt = float(np.sqrt(options.pin_weight)) if pin_slot is not None else 0.0
    pin_width = 6 if options.pin_orientation else 3
    use_pin = pin_slot is not None

    report = SolveReport(
        converged=False, iterations=0, final_residual_norm=float("inf"), max_residual=float("inf")
    )
    lam = _LAM0
    if use_pin and options.pin_weight > 0:
        # Joint rows are scaled by 1e3, while the mouse pin has weight 1.
        # Starting at assembly damping suppresses the free mechanism motion,
        # then every new mouse request resets that damping before it recovers.
        # Keep regularization proportional to the pin's squared row scale.
        joint_scale_squared = max(options.joint_weight ** 2, options.pin_weight)
        lam = max(_LAM_MIN, _LAM0 * options.pin_weight / joint_scale_squared)
    stopped_early = False
    numerical_failure = False
    accepted_pose_finite = True

    for iteration in range(options.max_iters):
        with trace.phase("evaluate"):
            evaluator.evaluate(
                model, workspace.origin, workspace.rotation, plan.joint_indices,
                workspace.residual, workspace.blocks, True,
            )
        joint_residual = workspace.residual[:n_joint]
        if n_joint and not np.isfinite(joint_residual).all():
            numerical_failure = True
            report.iterations = iteration + 1
            break
        jnorm = float(np.linalg.norm(joint_residual)) if n_joint else 0.0
        jmax = float(np.max(np.abs(joint_residual))) if n_joint else 0.0
        report.iterations = iteration + 1
        report.final_residual_norm = jnorm
        report.max_residual = jmax
        trace.iterations += 1

        if ncols == 0:
            report.converged = jmax < options.tol
            stopped_early = True
            break

        n_system = n_joint
        workspace.weighted[:n_joint] = joint_residual * options.joint_weight
        pin_values = None
        if use_pin:
            pin_values = workspace.weighted[n_joint:n_joint + pin_width]
            _write_pin(
                pin_values,
                workspace.origin[pin_slot],
                workspace.rotation[pin_slot],
                target_origin,
                target_rotation,
                options.pin_orientation,
                pin_sqrt,
            )
            n_system = n_joint + pin_width
        if not np.isfinite(workspace.weighted[:n_system]).all():
            numerical_failure = True
            break

        strategy = getattr(options, "strategy", None)
        use_blocks = strategy is not None and getattr(strategy, "uses_blocks", False)
        if not use_blocks:
            with trace.phase("assemble"):
                assemble_blocks(plan, model, workspace.blocks, workspace.jacobian)
            jacobian = workspace.jacobian[:n_joint, :ncols]
            jacobian *= options.joint_weight
            if use_pin:
                _write_pin_jacobian(
                    workspace.jacobian[n_joint:n_joint + pin_width, :ncols],
                    0,
                    pin_col,
                    options.pin_orientation,
                    pin_sqrt,
                )
                # The slice starts at the first pin row, so the local row is 0.
        with trace.phase("linear"):
            if use_blocks:
                step = strategy.solve_from_blocks(
                    plan,
                    model,
                    workspace.blocks,
                    workspace.weighted[:n_system],
                    lam,
                    joint_weight=options.joint_weight,
                    n_joint_rows=n_joint,
                    pin_column=pin_col,
                    pin_width=pin_width if use_pin else 0,
                    pin_sqrt=pin_sqrt if use_pin else 0.0,
                    delta_out=workspace.delta,
                )
            else:
                step = solve_dense_damped(
                    workspace.jacobian[:n_system, :ncols],
                    workspace.weighted[:n_system],
                    lam,
                    workspace.normal,
                    workspace.gradient,
                    workspace.delta,
                    workspace.system,
                )
        _record_linear_work(trace, step)
        trace.linear_residual = float(getattr(step, "linear_residual", 0.0))
        trace.linear_solver = str(getattr(step, "factorization", ""))
        if getattr(step, "failure_reason", ""):
            trace.failure_reason = step.failure_reason
        trace.last_step_norm = step.step_norm
        if not step.finite:
            trace.rejected_steps += 1
            lam = min(lam * 4.0, _LAM_MAX)
            continue

        if use_pin and options.pin_weight > 0:
            turns = workspace.delta[:ncols].reshape(-1, 6)[:, 3:]
            largest_turn = float(np.max(np.linalg.norm(turns, axis=1)))
            if largest_turn > _DRAG_MAX_ROTATION:
                fraction = _DRAG_MAX_ROTATION / largest_turn
                workspace.delta[:ncols] *= fraction
                step.step_norm *= fraction
                trace.last_step_norm = step.step_norm

        pin_settled = not use_pin or options.pin_weight <= 0 or np.linalg.norm(
            workspace.origin[pin_slot] - target_origin
        ) < options.tol
        if jmax < options.tol and pin_settled and step.step_norm < max(options.tol, 1e-10):
            report.converged = True
            stopped_early = True
            break

        old_norm = _weighted_norm(joint_residual, options.joint_weight, pin_values)
        with trace.phase("pose_update"):
            _apply_trial(workspace, slots, workspace.delta)
        if use_pin and options.pin_weight > 0:
            with trace.phase("drag_projection"):
                _correct_drag_trial(evaluator, model, workspace, plan, options, trace)
        with trace.phase("evaluate"):
            evaluator.evaluate(
                model, workspace.trial_origin, workspace.trial_rotation, plan.joint_indices,
                workspace.residual, workspace.blocks, False,
            )
        trial_joint = workspace.residual[:n_joint]
        trial_pin = None
        if use_pin:
            trial_pin = np.empty(pin_width, dtype=np.float64)
            _write_pin(
                trial_pin,
                workspace.trial_origin[pin_slot],
                workspace.trial_rotation[pin_slot],
                target_origin,
                target_rotation,
                options.pin_orientation,
                pin_sqrt,
            )
        new_norm = _weighted_norm(trial_joint, options.joint_weight, trial_pin)
        accept_trial = np.isfinite(new_norm) and new_norm < old_norm * (1.0 + _ACCEPT_REL)
        if accept_trial:
            _accept_trial(workspace, slots)
            trace.accepted_steps += 1
            lam = max(lam * 0.5, _LAM_MIN)
            if not poses_finite(workspace, plan.movable_body_ids, model):
                accepted_pose_finite = False
                numerical_failure = True
                break
        else:
            with trace.phase("pose_update"):
                _project_current(workspace, slots)
            trace.rejected_steps += 1
            lam = min(lam * 4.0, _LAM_MAX)

    trace.iteration_limit = (not stopped_early) and (not numerical_failure) and options.max_iters > 0
    if not numerical_failure:
        norms = _joint_norms(evaluator, model, workspace, plan)
        report.per_joint_residual = norms
        stacked = workspace.residual[:n_joint]
        report.final_residual_norm = float(np.linalg.norm(stacked)) if n_joint else 0.0
        report.max_residual = float(np.max(np.abs(stacked))) if n_joint else 0.0
        report.converged = report.converged or (
            report.max_residual < max(options.tol * 10.0, 1e-6)
        )
    else:
        report.converged = False
    report.moved_bodies = list(plan.movable_body_ids)
    if use_pin and pin_slot is not None and not numerical_failure:
        trace.pin_error = float(np.linalg.norm(workspace.origin[pin_slot] - target_origin))
    finite = accepted_pose_finite and poses_finite(workspace, plan.movable_body_ids, model)
    if not finite:
        numerical_failure = True
        report.converged = False
    trace.numerical_failure = trace.numerical_failure or numerical_failure
    return report, finite and not numerical_failure


def _analyze(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    plan: ComponentPlan,
    tol: float,
    trace: PhaseTrace,
) -> Tuple[List[str], Optional[int]]:
    with trace.phase("diagnostics"):
        if plan.ncols == 0 and plan.n_residual_rows == 0:
            return [], 0
        # Sparse steps need no dense Jacobian. Rank analysis allocates only
        # its own temporary Jacobian, and only when explicitly requested.
        jacobian_buffer = workspace.jacobian if workspace.dense_linear else np.zeros(
            (plan.n_residual_rows, plan.ncols), dtype=np.float64
        )
        if plan.n_residual_rows:
            evaluator.evaluate(
                model, workspace.origin, workspace.rotation, plan.joint_indices,
                workspace.residual, workspace.blocks, True,
            )
            assemble_blocks(plan, model, workspace.blocks, jacobian_buffer)
            jacobian = jacobian_buffer[:plan.n_residual_rows, :plan.ncols]
        else:
            jacobian = jacobian_buffer[:0, :plan.ncols]
        redundant, dof, _rank = analyze_jacobian(
            jacobian, plan.joint_names, plan.row_counts.tolist(), len(plan.movable_body_ids), tol
        )
    return redundant, dof


def solve_assembly(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    options: SolveOptions,
    trace: Optional[PhaseTrace] = None,
) -> EngineResult:
    trace = trace if trace is not None else PhaseTrace(enabled=options.trace)
    trace.backend = evaluator.name
    trace.formulation_version = model.formulation_version
    trace.grouping = model.grouping
    overall = SolveReport(True, 0, 0.0, 0.0)
    moved: List[int] = []
    finite = True
    if model.n_joints == 0:
        overall.message = "No joints to solve."
        trace.joint_feasible = True
        return EngineResult(overall, trace, moved, True)

    for plan in model.components:
        report, plan_ok = _solve_plan(
            evaluator, model, workspace, plan, options, trace,
            pin_body_id=None, pin_target=None,
        )
        finite = finite and plan_ok
        overall.iterations += report.iterations
        overall.converged = overall.converged and report.converged
        overall.per_joint_residual.update(report.per_joint_residual)
        overall.moved_bodies.extend(report.moved_bodies)
        moved.extend(report.moved_bodies)
        overall.final_residual_norm = max(overall.final_residual_norm, report.final_residual_norm)
        overall.max_residual = max(overall.max_residual, report.max_residual)
        if not plan_ok:
            break

    if finite and options.analyze:
        redundant, dof = _analyze(
            evaluator, model, workspace, model.analysis_plan, options.diagnostics_tol, trace
        )
        overall.redundant_joints = redundant
        overall.dof = dof
    if not finite:
        overall.converged = False
        overall.message = "Numerical failure"
        trace.numerical_failure = True
    else:
        overall.message = "Converged" if overall.converged else "Did not fully converge"
    trace.joint_feasible = finite and overall.max_residual < options.tol
    trace.copy_count = int(getattr(evaluator, "copy_count", 0))
    trace.zero_copy = getattr(evaluator, "zero_copy", None)
    return EngineResult(overall, trace, moved, finite)


def solve_drag(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    options: SolveOptions,
    dragged_body_id: int,
    target_origin: np.ndarray,
    target_rotation: np.ndarray,
    trace: Optional[PhaseTrace] = None,
) -> EngineResult:
    trace = trace if trace is not None else PhaseTrace(enabled=options.trace)
    trace.backend = evaluator.name
    trace.formulation_version = model.formulation_version
    trace.grouping = model.grouping
    from core.kinematics.prepared import component_for_body

    plan = component_for_body(model, int(dragged_body_id))
    if plan is None or plan.n_residual_rows == 0 or not plan.movable_body_ids:
        report = SolveReport(
            True, 0, 0.0, 0.0, message="Dragged body is not joint-constrained."
        )
        trace.joint_feasible = True
        return EngineResult(report, trace, [], True)

    report, finite = _solve_plan(
        evaluator, model, workspace, plan, options, trace,
        pin_body_id=int(dragged_body_id),
        pin_target=(target_origin, target_rotation),
    )
    if finite and options.analyze:
        redundant, dof = _analyze(
            evaluator, model, workspace, plan, options.diagnostics_tol, trace
        )
        report.redundant_joints = redundant
        report.dof = dof
    if not finite:
        report.converged = False
        report.message = "Numerical failure"
        trace.numerical_failure = True
        return EngineResult(report, trace, [], False)
    report.message = "Converged" if report.converged else "Did not fully converge"
    trace.joint_feasible = report.max_residual < options.tol
    trace.copy_count = int(getattr(evaluator, "copy_count", 0))
    trace.zero_copy = getattr(evaluator, "zero_copy", None)
    return EngineResult(report, trace, list(report.moved_bodies), True)


def diagnose_model(
    evaluator: ConstraintEvaluator,
    model: PreparedModel,
    workspace: Workspace,
    options: SolveOptions,
    trace: Optional[PhaseTrace] = None,
) -> EngineResult:
    """Rank the current pose without taking a step or changing workspace poses."""
    trace = trace if trace is not None else PhaseTrace(enabled=options.trace)
    trace.backend = evaluator.name
    trace.formulation_version = model.formulation_version
    trace.grouping = model.grouping
    report = SolveReport(True, 0, 0.0, 0.0, message="Diagnostics")
    if model.n_joints == 0:
        report.dof = 0
        report.message = "No joints to analyze."
        trace.joint_feasible = True
        return EngineResult(report, trace, [], True)
    norms = _joint_norms(evaluator, model, workspace, model.analysis_plan)
    report.per_joint_residual = norms
    stacked = workspace.residual[:model.analysis_plan.n_residual_rows]
    width = model.analysis_plan.n_residual_rows
    report.final_residual_norm = float(np.linalg.norm(stacked)) if width else 0.0
    report.max_residual = float(np.max(np.abs(stacked))) if width else 0.0
    redundant, dof = _analyze(
        evaluator, model, workspace, model.analysis_plan, options.diagnostics_tol, trace
    )
    report.redundant_joints = redundant
    report.dof = dof
    finite = poses_finite(workspace, model.body_ids, model)
    report.converged = finite and report.max_residual < options.tol
    trace.joint_feasible = bool(report.converged)
    if not finite:
        report.message = "Numerical failure"
        trace.numerical_failure = True
    return EngineResult(report, trace, [], finite)
