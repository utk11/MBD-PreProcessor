"""GUI-side commands: accept solver results, commit poses, and own the document lifecycle.

The numerical worker never touches this object. Results are checked against
the document revisions on the GUI thread, then committed and rendered.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import QObject
from PySide6.QtWidgets import QMessageBox

from core.kinematics.reports import RevisionStamp, SolveReport, commit_poses
from core.kinematics.solver import SolveRequest
from core.transforms import body_world_pose
from gui.solve_scheduler import SolveIntent, SolveScheduler, result_is_acceptable
from visualization.coordinator import RendererCoordinator


class _BodyId:
    def __init__(self, body_id: int):
        self.id = int(body_id)


class ApplicationController(QObject):
    """Routes document edits and solver results for one main window."""

    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.document = window.document
        self.coordinator: Optional[RendererCoordinator] = None
        self.scheduler = SolveScheduler(
            build_request=self._build_request,
            on_result=self._on_result,
            on_status=self._set_status,
        )
        self._assembly_message = False
        self._silent_requests = set()
        self._require_feasible_requests = set()
        self._locked_body_ids_by_request = {}

    def attach_viewer(self) -> None:
        self.coordinator = RendererCoordinator(
            self.window.body_renderer,
            self.window.display,
            on_body_changed=self.window._on_solver_body_changed,
        )

    def shutdown(self) -> bool:
        return self.scheduler.shutdown()

    def on_document_replaced(self) -> None:
        """Call after a new document generation is installed."""
        self.scheduler.reset_document()
        if self.coordinator is not None:
            self.coordinator.sync_baselines(self.document.bodies)
        if self.document.joints:
            self.scheduler.request_prewarm()

    def on_topology_changed(self) -> None:
        """Connectivity changed. In-flight results carry the old topology revision."""
        if self.document.joints:
            self.scheduler.request_prewarm()

    def on_drag_start(self, body_id: int) -> None:
        self.scheduler.begin_gesture()

    def submit_drag_target(self, body_id: int, position: np.ndarray) -> None:
        position = np.array(position, dtype=np.float64, copy=True)
        if int(body_id) not in self.document.constrained_body_ids():
            self._commit_external_pose(int(body_id), position)
            return
        self.scheduler.submit_drag(int(body_id), position)

    def finish_drag(self, body_id: int) -> None:
        if int(body_id) in self.document.constrained_body_ids():
            self.scheduler.finish_drag()

    def solve_assembly(self, notify: bool = True, require_feasible: bool = False,
                       reference_body_id: Optional[int] = None):
        if not self.document.bodies:
            QMessageBox.warning(self.window, "No Bodies", "Load a STEP file before solving.")
            return
        if not self.document.joints:
            QMessageBox.information(self.window, "No Joints", "Create joints before solving the assembly.")
            return
        if self.document.state is None:
            QMessageBox.warning(self.window, "No State", "Assembly state is not initialized.")
            return
        self._assembly_message = True
        intent = self.scheduler.submit_assembly()
        if intent is not None:
            if require_feasible:
                self._require_feasible_requests.add(intent.request_id)
            if reference_body_id is not None:
                self._locked_body_ids_by_request[intent.request_id] = (int(reference_body_id),)
            if not notify:
                self._silent_requests.add(intent.request_id)
            return intent.request_id

    def _commit_external_pose(self, body_id: int, position: np.ndarray) -> None:
        body = self.document.body_by_id(body_id)
        if body is None or self.document.state is None:
            return
        _origin, rotation = body_world_pose(body)
        self.document.state.set_body_pose(body_id, position, rotation)
        self.document.note_external_pose()
        if self.coordinator is not None:
            self.coordinator.apply([body_id])

    def _build_request(self, intent: SolveIntent) -> SolveRequest:
        poses = {}
        if self.document.state is not None:
            for body in self.document.bodies:
                pose = self.document.state.get_body_pose(body.id)
                if pose is None:
                    continue
                poses[int(body.id)] = (
                    np.array(pose.origin, dtype=np.float64, copy=True),
                    np.array(pose.rotation_matrix, dtype=np.float64, copy=True),
                )
        target_rotation = None
        if intent.kind == "drag":
            pair = poses.get(int(intent.body_id))
            target_rotation = np.eye(3) if pair is None else pair[1].copy()
        ground = self.window._body_pose_tuple(-1)
        stamp = RevisionStamp(
            document_generation=self.document.generation,
            topology_revision=self.document.topology_revision,
            marker_revision=self.document.marker_revision,
            external_pose_epoch=self.document.external_pose_epoch,
            pose_revision=self.document.pose_revision,
            request_id=intent.request_id,
            gesture_id=intent.gesture_id,
            epoch=intent.epoch,
            kind=intent.kind,
            finalize=intent.finalize,
            use_revisions=True,
        )
        max_iters = 80 if intent.kind == "assembly" else intent.max_iters
        tol = 1e-9 if intent.kind == "assembly" else intent.tol
        return SolveRequest(
            kind=intent.kind,
            bodies=[_BodyId(body.id) for body in self.document.bodies],
            joints=list(self.document.joints.values()),
            poses=poses,
            ground_pose=(
                np.array(ground[0], dtype=np.float64, copy=True),
                np.array(ground[1], dtype=np.float64, copy=True),
            ),
            stamp=stamp,
            dragged_body_id=intent.body_id,
            target_origin=None if intent.target is None else np.array(intent.target, copy=True),
            target_rotation=target_rotation,
            max_iters=max_iters,
            tol=tol,
            pin_weight=intent.pin_weight if intent.kind == "drag" else 0.0,
            pin_orientation=False,
            analyze=intent.kind in ("assembly", "diagnose"),
            locked_body_ids=self._locked_body_ids_by_request.get(intent.request_id, ()),
        )

    def _on_result(self, report: SolveReport) -> bool:
        notify = report.request_id not in self._silent_requests
        self._silent_requests.discard(report.request_id)
        require_feasible = report.request_id in self._require_feasible_requests
        self._require_feasible_requests.discard(report.request_id)
        self._locked_body_ids_by_request.pop(report.request_id, None)
        if report.kind == "prewarm":
            if report.finite:
                self._set_status("Solver ready")
            else:
                self._set_status(report.message or "Solver preparation failed")
            return False
        if report.kind == "assembly" and not self._acceptable(report):
            if notify and int(report.document_generation) == int(self.document.generation):
                self._show_failure(report)
            return False
        if not self._acceptable(report):
            return False
        if report.kind == "assembly" and require_feasible and not report.joint_feasible:
            self._set_status("Assembly did not satisfy all joint constraints; body poses were left unchanged.")
            if notify:
                QMessageBox.warning(
                    self.window,
                    "Assembly Not Feasible",
                    report.message or "The solver could not satisfy every joint. Body poses were left unchanged.",
                )
            return False
        if report.kind == "diagnose":
            self._show_diagnostics(report)
            return True
        if self.document.state is None:
            return False
        changed = commit_poses(self.document.state, report)
        self.document.note_pose_commit()
        if self.coordinator is not None and changed:
            self.coordinator.apply(changed)
        settled_drag = report.kind == "drag" and (
            report.finalize
            or report.request_id == self.scheduler.queue.diagnose_after_id
        )
        if report.kind == "assembly":
            self._show_assembly(report, notify=notify)
            self._notify_settled(report)
        elif settled_drag:
            self._show_drag_status(report)
            self._notify_settled(report)
        return True

    def _acceptable(self, report: SolveReport) -> bool:
        return result_is_acceptable(
            report,
            generation=self.document.generation,
            topology=self.document.topology_revision,
            marker=self.document.marker_revision,
            external=self.document.external_pose_epoch,
            pose=self.document.pose_revision,
            epoch=self.scheduler.queue.epoch,
            gesture=self.scheduler.queue.current_gesture,
            last_committed=self.scheduler.queue.last_committed_request_id,
        )

    def _set_status(self, message: str) -> None:
        if hasattr(self.window, "statusBar"):
            self.window.statusBar().showMessage(message, 8000)

    def _show_failure(self, report: SolveReport) -> None:
        self._set_status(report.message or "Solve failed")
        QMessageBox.critical(
            self.window,
            "Solve Error",
            report.message or "The assembly solve failed.",
        )

    def _show_assembly(self, report: SolveReport, notify: bool = True) -> None:
        residual_lines = [
            f"  {name}: {residual:.3e}"
            for name, residual in sorted(report.per_joint_residual.items())
        ]
        residual_txt = "\n".join(residual_lines) if residual_lines else "  (none)"
        redundant = ", ".join(report.redundant_joints) if report.redundant_joints else "none"
        feasible = "yes" if report.joint_feasible else "no"
        target = "n/a" if report.target_error is None else f"{report.target_error:.3e}"
        text = (
            f"{'Converged' if report.converged else 'Did not fully converge'}\n"
            f"Iterations: {report.iterations}\n"
            f"Max residual: {report.max_residual:.3e}\n"
            f"Joint constraints feasible: {feasible}\n"
            f"Target error: {target}\n"
            f"Estimated DOF: {report.dof}\n"
            f"Redundant joints: {redundant}\n\n"
            f"Per-joint residual norms:\n{residual_txt}"
        )
        self._set_status(
            f"Solve {'OK' if report.converged else 'partial'} | "
            f"DOF={report.dof} | max res={report.max_residual:.2e}"
        )
        if notify:
            QMessageBox.information(self.window, "Solve Assembly", text)

    def _show_drag_status(self, report: SolveReport) -> None:
        target = "n/a" if report.target_error is None else f"{report.target_error:.2e}"
        feasible = "yes" if report.joint_feasible else "no"
        self._set_status(
            f"Drag residual {report.max_residual:.2e} | joints feasible: {feasible} | target error {target}"
        )

    def _notify_settled(self, report: SolveReport) -> None:
        callback = getattr(self.window, "_on_solver_settled", None)
        if callback is not None:
            callback(report)

    def _show_diagnostics(self, report: SolveReport) -> None:
        redundant = ", ".join(report.redundant_joints) if report.redundant_joints else "none"
        self._set_status(
            f"Settled | DOF={report.dof} | redundant: {redundant} | "
            f"max residual {report.max_residual:.2e}"
        )
