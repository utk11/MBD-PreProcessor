"""GUI selection, request ownership, cancellation, and real worker switching."""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from main import MainWindow
from PySide6.QtWidgets import QApplication, QMainWindow

from core.assembly_document import AssemblyDocument
from core.kinematics.reports import SolveReport, apply_stamp
from gui.application_controller import ApplicationController
from gui.solve_scheduler import SolveQueue, SolveScheduler
from gui.solver_selector import SOLVER_CHOICES
from tests.kinematics_fixtures import build_pendulum, GROUND_POSE
from tests.test_refactor_regressions import spin_until


class SolverSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def make_window(self):
        window = QMainWindow()
        bodies, joints, state = build_pendulum(True)
        window.document = AssemblyDocument()
        window.document.replace_bodies(bodies)
        window.document.state = state
        for joint in joints:
            window.document.add_joint(joint)
        window._body_pose_tuple = lambda body_id: GROUND_POSE
        window._on_solver_settled = Mock()
        return window

    def test_dropdown_snapshots_selection_and_cancels_old_work(self):
        queue = SolveQueue()
        scheduler = SimpleNamespace(
            queue=queue, reset_document=queue.reset_document,
            request_prewarm=Mock(side_effect=queue.request_prewarm),
        )
        window = self.make_window()
        with patch("gui.application_controller.SolveScheduler", return_value=scheduler):
            window.controller = ApplicationController(window)
        MainWindow.create_solver_toolbar(window)
        selector = window.solver_selector
        self.assertEqual(selector.currentData(), "dense")
        self.assertEqual(selector.count(), 4)
        self.assertEqual([selector.itemData(i) for i in range(4)], [key for key, _ in SOLVER_CHOICES])
        old_intent = queue.submit_assembly()
        queue.pop_for_dispatch()
        old_request = window.controller._build_request(old_intent)
        old = SolveReport(True, 1, 0.0, 0.0, finite=True)
        apply_stamp(old, old_request.stamp)
        before = window.document.state.get_body_pose(1).origin.copy()
        window.controller._show_failure = Mock()
        for key, _label in SOLVER_CHOICES[1:]:
            selector.setCurrentIndex(selector.findData(key))
            self.assertEqual(window.controller.linear_solver_name, key)
            intent = queue.submit_drag(1, before)
            request = window.controller._build_request(intent)
            self.assertEqual(request.linear_solver, key)
        self.assertEqual(old_request.linear_solver, "dense")
        self.assertFalse(window.controller._on_result(old))
        window.controller._show_failure.assert_not_called()
        np.testing.assert_array_equal(window.document.state.get_body_pose(1).origin, before)
        self.assertTrue(queue.prewarm_needed)
        self.assertEqual(scheduler.request_prewarm.call_count, 3)
        with self.assertRaises(ValueError):
            window.controller.set_linear_solver("invalid")
        self.assertEqual(window.controller.linear_solver_name, "lsmr")
        # Returning to dense also snapshots the selected method for assembly.
        selector.setCurrentIndex(selector.findData("dense"))
        self.assertEqual(window.controller._build_request(queue.submit_assembly()).linear_solver, "dense")
        window.close()

    def test_old_assembly_completion_keeps_new_assembly_protected_from_drag(self):
        queue = SolveQueue()
        queue.submit_assembly()
        old = queue.pop_for_dispatch()
        queue.reset_document()  # The user switches methods while old work runs.
        latest = queue.submit_assembly()
        scheduler = SimpleNamespace(
            _closed=False, queue=queue, on_result=Mock(return_value=False),
            on_status=Mock(), report_processed=Mock(), pump=Mock(),
        )
        SolveScheduler._on_worker_result(
            scheduler, SolveReport(True, 1, 0.0, 0.0, epoch=old.epoch, kind="assembly")
        )
        self.assertTrue(queue.drag_suspended)
        self.assertIsNone(queue.submit_drag(1, np.zeros(3)))
        self.assertIs(queue.pending, latest)

    def test_real_worker_uses_each_choice_for_assembly_and_drag_without_recompile(self):
        window = self.make_window()
        window.controller = ApplicationController(window)
        MainWindow.create_solver_toolbar(window)
        controller = window.controller
        scheduler = controller.scheduler
        processed = []
        scheduler.report_processed.connect(lambda report, accepted: processed.append((report, accepted)))

        def idle():
            return (scheduler.queue.active is None and scheduler.queue.pending is None
                    and not scheduler.queue.prewarm_needed and not scheduler.queue.diagnose_needed)

        try:
            scheduler.request_prewarm()
            self.assertTrue(spin_until(idle, 30000))
            session = scheduler._worker._solver
            compiled = session.evaluator.compile_count
            self.assertGreater(compiled, 0)
            for key in ("superlu", "cg", "lsmr", "dense"):
                with self.subTest(method=key):
                    window.solver_selector.setCurrentIndex(window.solver_selector.findData(key))
                    self.assertTrue(spin_until(idle, 30000))
                    self.assertIs(scheduler._worker._solver, session)
                    self.assertEqual(session.linear_solver_name, key)
                    window.document.state.set_body_pose(1, [0.02, -1.0, 0.01], np.eye(3))
                    window.document.note_external_pose()
                    request_id = controller.solve_assembly(notify=False)
                    self.assertTrue(spin_until(idle, 30000))
                    report, accepted = next(pair for pair in processed if pair[0].request_id == request_id)
                    self.assertTrue(accepted)
                    self.assertTrue(report.converged)
                    self.assertLess(report.max_residual, 1e-6)
                    self.assertEqual(session.linear_solver_name, key)
                    pose = window.document.state.get_body_pose(1)
                    controller.on_drag_start(1)
                    controller.submit_drag_target(1, pose.origin + np.array([0.03, 0.01, 0.0]))
                    controller.finish_drag(1)
                    self.assertTrue(spin_until(idle, 30000))
                    drags = [pair for pair in processed if pair[0].kind == "drag"]
                    self.assertTrue(drags[-1][1])
                    self.assertTrue(drags[-1][0].finite)
                    self.assertLess(drags[-1][0].max_residual, 1e-6)
                    self.assertEqual(session.linear_solver_name, key)
                    self.assertEqual(session.evaluator.compile_count, compiled)
                    self.assertEqual(session._workspace.jacobian.size == 0, key != "dense")
        finally:
            scheduler.shutdown()
            self.assertTrue(spin_until(lambda: scheduler.is_stopped, 30000))
            window.close()


if __name__ == "__main__":
    unittest.main()
