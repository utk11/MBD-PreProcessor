"""One active solve and one replaceable pending target, on a single worker thread.

The worker owns the JAX session. It never reads widgets or writes the live
document. Mouse targets replace the pending request instead of queueing.
A finished result is delivered even when a newer target is already waiting, so
fast input still produces visible motion. The final mouse target is kept.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal, Slot

from core.kinematics.reports import SolveReport


@dataclass
class SolveIntent:
    kind: str
    request_id: int
    gesture_id: int
    epoch: int
    finalize: bool = False
    body_id: Optional[int] = None
    target: Optional[np.ndarray] = None
    analyze: bool = False
    max_iters: int = 12
    tol: float = 1e-6
    pin_weight: float = 1.0
    pin_orientation: bool = False


def result_is_acceptable(
    report: SolveReport,
    *,
    generation: int,
    topology: int,
    marker: int,
    external: int,
    pose: int,
    epoch: int,
    gesture: int,
    last_committed: int,
) -> bool:
    """True when a worker result may still be committed or displayed."""
    if not report.finite:
        return False
    if int(report.document_generation) != int(generation):
        return False
    if int(report.epoch) != int(epoch):
        return False
    if int(report.request_id) <= int(last_committed):
        return False
    if int(report.topology_revision) != int(topology):
        return False
    if int(report.marker_revision) != int(marker):
        return False
    if int(report.external_pose_epoch) != int(external):
        return False
    if report.kind == "diagnose":
        return (
            int(report.pose_revision) == int(pose)
            and int(report.gesture_id) == int(gesture)
        )
    if report.kind == "drag" and int(report.gesture_id) != int(gesture):
        return False
    return True


class SolveQueue:
    """Scheduling policy with no Qt dependency."""

    def __init__(self):
        self.epoch = 0
        self.next_request_id = 1
        self.gesture_id = 0
        self.current_gesture = 0
        self.active: Optional[SolveIntent] = None
        self.pending: Optional[SolveIntent] = None
        self.drag_suspended = False
        self.prewarm_needed = False
        self.diagnose_needed = False
        self.diagnose_after_id: Optional[int] = None
        self.last_committed_request_id = 0

    def _make(self, kind: str, **kwargs) -> SolveIntent:
        intent = SolveIntent(
            kind=kind,
            request_id=self.next_request_id,
            gesture_id=kwargs.pop("gesture_id", self.current_gesture),
            epoch=self.epoch,
            **kwargs,
        )
        self.next_request_id += 1
        return intent

    def begin_gesture(self) -> int:
        self.gesture_id += 1
        self.current_gesture = self.gesture_id
        self.diagnose_needed = False
        if self.pending is not None and self.pending.kind == "drag":
            self.pending = None
        return self.current_gesture

    def submit_drag(self, body_id: int, target: np.ndarray, **limits) -> Optional[SolveIntent]:
        """Replace the single pending drag target. A new target does not queue behind old ones."""
        if self.drag_suspended:
            return None
        intent = self._make(
            "drag",
            body_id=int(body_id),
            target=np.array(target, dtype=np.float64, copy=True),
            analyze=False,
            **limits,
        )
        self.pending = intent
        return intent

    def mark_final(self) -> None:
        """Keep the latest drag target and ask for diagnostics after it settles."""
        if self.pending is not None and self.pending.kind == "drag":
            self.pending.finalize = True
            return
        if self.active is not None and self.active.kind == "drag":
            self.diagnose_after_id = self.active.request_id
            return
        self.diagnose_needed = True

    def submit_assembly(self, max_iters: int = 80, tol: float = 1e-9) -> SolveIntent:
        """Drop drag work. An in-flight drag keeps its old epoch and cannot commit."""
        self.epoch += 1
        self.drag_suspended = True
        self.diagnose_needed = False
        self.pending = self._make("assembly", analyze=True, max_iters=max_iters, tol=tol)
        return self.pending

    def request_prewarm(self) -> None:
        self.prewarm_needed = True

    def reset_document(self) -> None:
        """Invalidate queued work. The request already running is left to finish."""
        self.epoch += 1
        self.current_gesture += 1
        self.pending = None
        self.diagnose_needed = False
        self.diagnose_after_id = None
        self.drag_suspended = False
        self.prewarm_needed = False
        self.last_committed_request_id = 0

    def pop_for_dispatch(self) -> Optional[SolveIntent]:
        if self.active is not None:
            return None
        if self.prewarm_needed:
            self.prewarm_needed = False
            self.active = self._make("prewarm")
            return self.active
        if self.pending is not None:
            self.active = self.pending
            self.pending = None
            return self.active
        if self.diagnose_needed and not self.drag_suspended:
            self.diagnose_needed = False
            self.active = self._make("diagnose", analyze=True)
            return self.active
        return None

    def note_accepted(self, request_id: int) -> None:
        if int(request_id) > self.last_committed_request_id:
            self.last_committed_request_id = int(request_id)


class SolveScheduler(QObject):
    """Qt owner of the queue and the long-lived numerical thread."""

    _submit = Signal(object)
    shutdown_finished = Signal()
    report_processed = Signal(object, bool)

    def __init__(self, build_request, on_result, on_status):
        super().__init__()
        self.queue = SolveQueue()
        self.build_request = build_request
        self.on_result = on_result
        self.on_status = on_status
        self._thread = QThread()
        self._worker = _NumericalWorker()
        self._worker.moveToThread(self._thread)
        self._submit.connect(self._worker.handle_request)
        self._worker.result_ready.connect(self._on_worker_result)
        self._worker.status_changed.connect(self._on_status)
        self._thread.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self.shutdown_finished)
        self._thread.start()
        self._closed = False

    def begin_gesture(self) -> int:
        return self.queue.begin_gesture()

    def submit_drag(self, body_id: int, target: np.ndarray) -> None:
        if self._closed:
            return
        self.queue.submit_drag(body_id, target)
        self.pump()

    def finish_drag(self) -> None:
        if self._closed:
            return
        self.queue.mark_final()
        self.pump()

    def submit_assembly(self):
        if self._closed or self.queue.drag_suspended:
            return
        intent = self.queue.submit_assembly()
        self.on_status("Solving assembly...")
        self.pump()
        return intent

    def request_prewarm(self) -> None:
        if self._closed:
            return
        self.queue.request_prewarm()
        self.on_status("Preparing solver...")
        self.pump()

    def reset_document(self) -> None:
        self.queue.reset_document()

    def pump(self) -> None:
        if self._closed:
            return
        intent = self.queue.pop_for_dispatch()
        if intent is None:
            return
        try:
            request = self.build_request(intent)
        except Exception as exc:
            self.queue.active = None
            self.on_status(f"Solve request failed: {exc}")
            return
        self._submit.emit(request)

    @property
    def is_stopped(self) -> bool:
        return not self._thread.isRunning()

    def shutdown(self) -> bool:
        """Request cleanup after active work; report whether closure is safe.

        The GUI keeps the scheduler alive until shutdown_finished. The worker
        releases its session before quitting its own event loop.
        """
        if self._closed:
            return self.is_stopped
        self._closed = True
        self.queue.epoch += 1
        self.queue.pending = None
        self.queue.diagnose_needed = False
        self._submit.emit(None)
        return self.is_stopped

    @Slot(str)
    def _on_status(self, message: str) -> None:
        if not self._closed:
            self.on_status(message)

    @Slot(object)
    def _on_worker_result(self, report: SolveReport) -> None:
        if self._closed:
            self.queue.active = None
            return
        finished = self.queue.active
        self.queue.active = None
        if finished is not None and finished.kind == "assembly":
            self.queue.drag_suspended = False
        accepted = False
        try:
            accepted = bool(self.on_result(report))
        except Exception as exc:
            self.on_status(f"Could not apply solve result: {exc}")
        if accepted:
            self.queue.note_accepted(report.request_id)
            if (
                finished is not None
                and finished.kind == "drag"
                and (finished.finalize or finished.request_id == self.queue.diagnose_after_id)
            ):
                self.queue.diagnose_needed = True
        self.queue.diagnose_after_id = None
        self.report_processed.emit(report, accepted)
        self.pump()


class _NumericalWorker(QObject):
    """Owns the solver session. One document generation, one session."""

    result_ready = Signal(object)
    status_changed = Signal(str)

    def __init__(self):
        super().__init__()
        self._solver = None
        self._generation = None
        self._closing = False

    @Slot(object)
    def handle_request(self, request) -> None:
        if request is None:
            self._closing = True
            try:
                self._release()
            finally:
                self.thread().quit()
            return
        if self._closing:
            return
        try:
            self._ensure_session(request)
            if request.kind == "prewarm":
                self.status_changed.emit("Preparing solver...")
            report = self._solver.solve_request(request)
            self.result_ready.emit(report)
        except Exception as exc:
            from core.kinematics.reports import apply_stamp
            report = SolveReport(
                converged=False,
                iterations=0,
                final_residual_norm=float("inf"),
                max_residual=float("inf"),
                message=str(exc),
                finite=False,
            )
            apply_stamp(report, getattr(request, "stamp", None))
            self.result_ready.emit(report)

    def _ensure_session(self, request) -> None:
        from core.kinematics.solver import KinematicSolver

        generation = int(request.stamp.document_generation)
        if self._solver is None or self._generation != generation:
            self._release()
            self._solver = KinematicSolver(
                request.bodies,
                request.joints,
                state=None,
                ground_id=-1,
                ground_pose=request.ground_pose,
                locked_body_ids=request.locked_body_ids,
            )
            self._generation = generation
            return
        self._solver.bodies = list(request.bodies)
        self._solver.joints = list(request.joints)
        self._solver.ground_pose = request.ground_pose
        self._solver.locked_body_ids = set(int(body_id) for body_id in request.locked_body_ids)
        self._solver.locked_body_ids.add(self._solver.ground_id)

    def _release(self) -> None:
        if self._solver is not None:
            self._solver.release()
            self._solver = None
        self._generation = None
