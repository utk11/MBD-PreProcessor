"""Regressions for restored documents, solver caches, and Qt worker lifetime."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import sys
import tempfile
from threading import Event, get_ident
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PySide6.QtCore import QCoreApplication, QEventLoop, QTimer

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.assembly_document import AssemblyDocument
from core.data_structures import Frame
from core.kinematics import KinematicSolver
from core.kinematics.prepared import prepare
from core.kinematics.reports import SolveReport
from core.kinematics.workspace import create_workspace, ensure_workspace, validate_workspace
from core.project_store import (
    LoadedProject, ProjectValidationError, prepare_import_document,
    read_project, save_project, validate_against_bodies,
)
from core.transforms import attachment_reference_frame, attachment_world_frame
from gui.solve_scheduler import SolveScheduler, result_is_acceptable
from main import MainWindow
from tests.kinematics_fixtures import build_pendulum, make_body, make_state, Rz


def loaded_project(**records):
    data = {"bodies": [{"id": 1}], "joints": [], "frames": [], "forces": [], "torques": []}
    data.update(records)
    return LoadedProject(2, "part.step", None, None, 1.0, data)


def spin_until(predicate, timeout_ms=5000):
    """Process Qt signals while waiting, without blocking the GUI event loop."""
    if predicate():
        return True
    loop = QEventLoop()
    poll = QTimer()
    poll.setInterval(5)
    poll.timeout.connect(lambda: loop.quit() if predicate() else None)
    timeout = QTimer()
    timeout.setSingleShot(True)
    timeout.timeout.connect(loop.quit)
    poll.start()
    timeout.start(timeout_ms)
    loop.exec()
    poll.stop()
    timeout.stop()
    return bool(predicate())


class BindingTests(unittest.TestCase):
    def test_same_topology_replacement_refreshes_markers_without_recompile(self):
        bodies, joints, state = build_pendulum(False)
        solver = KinematicSolver(bodies, joints, state)
        try:
            original = solver.solve_assembly(max_iters=0, analyze=False)
            self.assertAlmostEqual(original.max_residual, 0.0)
            compiled = solver.evaluator.compile_count
            replacement = copy.deepcopy(joints[0])
            replacement.marker1.origin += np.array([0.25, 0.0, 0.0])
            solver.joints = [replacement]
            solver.invalidate("topology")
            changed = solver.solve_assembly(max_iters=0, analyze=False)
            self.assertAlmostEqual(changed.max_residual, 0.25)
            self.assertEqual(solver.evaluator.compile_count, compiled)
            solver.joints = joints
            solver.invalidate("topology")
            restored = solver.solve_assembly(max_iters=0, analyze=False)
            self.assertAlmostEqual(restored.max_residual, 0.0)
        finally:
            solver.release()


class RestorationTests(unittest.TestCase):
    def test_late_restoration_failure_keeps_the_open_document_and_view(self):
        old = AssemblyDocument()
        body = make_body(7, [3.0, 4.0, 5.0])
        old.replace_bodies([body])
        old.state = make_state([body])
        old.begin_generation()
        state = old.state
        # Record validation succeeds, but adding a motor to a fixed joint fails.
        invalid = loaded_project(joints=[{
            "name": "invalid_motor", "type": "FIXED", "body1_id": -1, "body2_id": 1,
            "is_motorized": True, "motor_type": "VELOCITY", "motor_value": 1.0,
        }])
        window = SimpleNamespace(
            _close_requested=False, _load_generation=2, _pending_project=invalid,
            document=old, _clear_ui_for_new_load=Mock(),
        )
        payload = SimpleNamespace(
            generation=2, bodies=[make_body(1, [0.0, 0.0, 0.0])],
            unit_scale=1.0, filepath="part.step",
        )
        with patch("main.QMessageBox.critical") as error:
            MainWindow.on_load_result(window, payload)
        error.assert_called_once()
        window._clear_ui_for_new_load.assert_not_called()
        self.assertIs(old.state, state)
        self.assertEqual(old.bodies, [body])
        self.assertEqual(old.generation, 1)
        np.testing.assert_allclose(old.state.get_body_pose(7).origin, [3.0, 4.0, 5.0])

    def test_saved_membership_survives_cad_reimport(self):
        document = AssemblyDocument()
        document.replace_bodies([make_body(1, [10.0, 0.0, 0.0]), make_body(2, [20.0, 0.0, 0.0])])
        document.state = make_state(document.bodies)
        document.delete_bodies([2])
        document.bodies[0].visible = False
        document.state.set_body_pose(1, [2.0, 3.0, 0.0], Rz(90))
        with tempfile.TemporaryDirectory() as folder:
            step = os.path.join(folder, "part.step")
            Path(step).write_bytes(b"test CAD")
            project = os.path.join(folder, "assembly.mbdp")
            save_project(project, document, step)
            loaded = read_project(project)
            candidate, notes, baselines = prepare_import_document(
                [make_body(1, [10.0, 0.0, 0.0]), make_body(2, [20.0, 0.0, 0.0])],
                1.0, step, loaded,
            )
        self.assertEqual([body.id for body in candidate.bodies], [1])
        self.assertIsNone(candidate.state.get_body_pose(2))
        self.assertFalse(candidate.bodies[0].visible)
        self.assertEqual(notes, [])
        np.testing.assert_allclose(candidate.state.get_body_pose(1).origin, [2.0, 3.0, 0.0])
        np.testing.assert_allclose(baselines[1][0], [10.0, 0.0, 0.0])

    def test_empty_saved_assembly_and_legacy_membership(self):
        candidate, _, _ = prepare_import_document(
            [make_body(1, [0.0, 0.0, 0.0])], 1.0, "part.step", loaded_project(bodies=[]),
        )
        self.assertEqual(candidate.bodies, [])
        self.assertEqual(candidate.state.body_poses, {})
        legacy = LoadedProject(1, "part.step", None, None, 1.0, {"joints": [], "frames": []})
        candidate, _, _ = prepare_import_document(
            [make_body(1, [0.0, 0.0, 0.0]), make_body(2, [1.0, 0.0, 0.0])], 1.0, "part.step", legacy,
        )
        self.assertEqual([body.id for body in candidate.bodies], [1, 2])

    def test_references_to_deleted_members_are_rejected(self):
        loaded = loaded_project(joints=[{
            "name": "dangling", "type": "FIXED", "body1_id": 1, "body2_id": 2,
        }])
        with self.assertRaises(ProjectValidationError):
            validate_against_bodies(loaded, [1, 2])

    def test_install_preserves_widget_aliases_and_advances_generation(self):
        live = AssemblyDocument()
        live.begin_generation()
        aliases = {name: getattr(live, name) for name in (
            "bodies", "joints", "frames", "frame_to_body", "frame_coordinates", "forces", "torques",
        )}
        candidate, _, _ = prepare_import_document([make_body(1, [1.0, 0.0, 0.0])], 0.001, "new.step")
        candidate.add_frame(Frame(name="local"), 1, "body_local")
        live.install(candidate)
        for name, alias in aliases.items():
            self.assertIs(getattr(live, name), alias)
        self.assertEqual(live.generation, 2)
        self.assertEqual(live.frames_by_body, {1: ["local"]})
        self.assertIs(live.bodies[0].state, live.state)

    def test_restored_geometry_uses_imported_baseline_and_saved_visibility(self):
        from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
        from visualization.body_renderer import BodyRenderer
        body = make_body(1, [10.0, 0.0, 0.0])
        body.shape = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
        state = make_state([body])
        state.set_body_pose(1, [2.0, 3.0, 0.0], Rz(90))
        body.visible = False
        context = Mock()
        renderer = BodyRenderer(SimpleNamespace(Context=context))
        renderer.display_bodies([body], base_poses={1: (np.array([10.0, 0.0, 0.0]), np.eye(3))})
        trsf = renderer.body_ais_shapes[1].LocalTransformation()
        translation = trsf.TranslationPart()
        np.testing.assert_allclose([translation.X(), translation.Y(), translation.Z()], [2.0, -7.0, 0.0])
        np.testing.assert_allclose(renderer._base_poses[1].origin, [10.0, 0.0, 0.0])
        context.Erase.assert_called_once_with(renderer.body_ais_shapes[1], False)


class FrameTests(unittest.TestCase):
    def test_dialogs_receive_current_world_frames_in_all_coordinate_conventions(self):
        document = AssemblyDocument()
        body = make_body(1, [10.0, 0.0, 0.0])
        document.replace_bodies([body])
        document.state = make_state([body])
        document.state.set_body_pose(1, [2.0, 3.0, 0.0], Rz(90))
        document.add_frame(Frame([11.0, 0.0, 0.0], name="geometry"), 1, "reference_geometry")
        document.add_frame(Frame([1.0, 0.0, 0.0], name="local"), 1, "body_local")
        document.add_frame(Frame([7.0, 8.0, 0.0], name="world"), 1, "world")
        window = SimpleNamespace(
            world_frame=Frame(name="World Frame"), bodies=document.bodies,
            created_frames=document.frames, frame_to_body_map=document.frame_to_body,
            document=document, ground_body=make_body(-1, [0.0, 0.0, 0.0]),
        )
        frames = {frame.name: frame for frame in MainWindow._available_world_frames(window)}
        np.testing.assert_allclose(frames[body.local_frame.name].origin, [2.0, 3.0, 0.0])
        for name in ("geometry", "local"):
            np.testing.assert_allclose(frames[name].origin, [2.0, 4.0, 0.0])
            np.testing.assert_allclose(frames[name].rotation_matrix, Rz(90))
        np.testing.assert_allclose(frames["world"].origin, [7.0, 8.0, 0.0])
        frames["local"].origin[:] = 99.0
        np.testing.assert_allclose(document.frames["local"].origin, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(body.local_frame.origin, [10.0, 0.0, 0.0])

    def test_local_rendering_frame_and_world_frame_agree_with_rotated_reference(self):
        body = make_body(1, [10.0, 0.0, 0.0], Rz(90))
        state = make_state([body])
        state.set_body_pose(1, [2.0, 3.0, 0.0], Rz(180))
        local = Frame([1.0, 0.0, 0.0], Rz(15), "local")
        reference = attachment_reference_frame(local, body, "body_local")
        np.testing.assert_allclose(reference.origin, [10.0, 1.0, 0.0])
        world = attachment_world_frame(reference, body, "reference_geometry")
        direct = attachment_world_frame(local, body, "body_local")
        np.testing.assert_allclose(world.origin, direct.origin)
        np.testing.assert_allclose(world.rotation_matrix, direct.rotation_matrix)
        np.testing.assert_allclose(direct.origin, [1.0, 3.0, 0.0])


class SparseWorkspaceTests(unittest.TestCase):
    def test_sparse_workspaces_omit_dense_matrices_and_can_switch_back(self):
        bodies, joints, _ = build_pendulum(False)
        model = prepare(bodies, joints)
        sparse = create_workspace(model, dense_linear=False)
        validate_workspace(sparse)
        for name in ("jacobian", "normal", "system", "gradient"):
            self.assertEqual(getattr(sparse, name).nbytes, 0)
        self.assertIs(ensure_workspace(sparse, model, dense_linear=False), sparse)
        dense = ensure_workspace(sparse, model)
        self.assertIsNot(dense, sparse)
        self.assertGreater(dense.jacobian.nbytes, 0)
        validate_workspace(dense)

    def test_sparse_full_solve_drag_and_diagnostics_need_no_persistent_dense_matrices(self):
        for strategy in ("superlu", "lsmr"):
            with self.subTest(strategy=strategy):
                bodies, joints, state = build_pendulum(True)
                solver = KinematicSolver(bodies, joints, state, linear_solver=strategy)
                try:
                    solver.prewarm()
                    report = solver.solve_assembly(max_iters=30, tol=1e-8, analyze=True)
                    self.assertTrue(report.finite)
                    self.assertTrue(report.converged)
                    self.assertEqual(report.dof, 1)
                    pose = state.get_body_pose(1)
                    drag = solver.solve_drag(1, pose.origin + [0.02, 0.01, 0.0], analyze=False)
                    self.assertTrue(drag.finite)
                    self.assertIsNone(drag.dof)
                    for name in ("jacobian", "normal", "system", "gradient"):
                        self.assertEqual(getattr(solver._workspace, name).nbytes, 0)
                    validate_workspace(solver._workspace)
                finally:
                    solver.release()


class ResultAndShutdownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_diagnostics_reject_every_stale_input_revision(self):
        report = SolveReport(
            True, 0, 0.0, 0.0, finite=True, kind="diagnose", document_generation=1,
            topology_revision=2, marker_revision=3, external_pose_epoch=4,
            pose_revision=5, epoch=6, gesture_id=7, request_id=8,
        )
        current = dict(generation=1, topology=2, marker=3, external=4, pose=5, epoch=6, gesture=7, last_committed=0)
        self.assertTrue(result_is_acceptable(report, **current))
        for revision in ("generation", "topology", "marker", "external", "pose", "epoch", "gesture"):
            with self.subTest(revision=revision):
                changed = {**current, revision: current[revision] + 1}
                self.assertFalse(result_is_acceptable(report, **changed))

    def test_shutdown_keeps_worker_alive_until_active_work_and_release_finish(self):
        started, allow_finish = Event(), Event()
        released = []
        finished = []
        worker_threads = []
        delivered = Mock()

        def solve(_request):
            worker_threads.append(get_ident())
            started.set()
            if not allow_finish.wait(5.0):
                raise RuntimeError("Test worker was not released")
            return SolveReport(True, 0, 0.0, 0.0)

        scheduler = SolveScheduler(lambda intent: SimpleNamespace(kind=intent.kind), delivered, Mock())
        scheduler._worker._solver = SimpleNamespace(
            solve_request=solve, release=lambda: released.append(get_ident()),
        )
        scheduler._worker._ensure_session = lambda request: None
        scheduler.shutdown_finished.connect(lambda: finished.append(True))
        try:
            scheduler.submit_assembly()
            self.assertTrue(spin_until(started.is_set))
            before = time.perf_counter()
            self.assertFalse(scheduler.shutdown())
            self.assertLess(time.perf_counter() - before, 0.5)
            self.assertFalse(scheduler.is_stopped)
            self.assertEqual(released, [])
            allow_finish.set()
            self.assertTrue(spin_until(lambda: bool(finished)))
            self.assertTrue(scheduler.is_stopped)
            self.assertEqual(released, worker_threads)
            self.assertNotEqual(released[0], get_ident())
            delivered.assert_not_called()
            self.assertTrue(scheduler.shutdown())
        finally:
            allow_finish.set()
            scheduler.shutdown()
            scheduler._thread.wait(5000)

    def test_window_close_waits_for_solver_and_import_workers(self):
        solver = SimpleNamespace(is_stopped=False)
        importer = Mock()
        importer.isRunning.return_value = True
        controller = SimpleNamespace(scheduler=solver, shutdown=Mock())
        window = SimpleNamespace(
            _close_requested=False, _drag_update_timer=Mock(), setEnabled=Mock(),
            controller=controller, _load_workers={importer}, statusBar=Mock(), close=Mock(),
        )
        event = Mock()
        MainWindow.closeEvent(window, event)
        event.ignore.assert_called_once()
        controller.shutdown.assert_called_once()
        MainWindow._finish_close_if_ready(window)
        window.close.assert_not_called()
        solver.is_stopped = True
        MainWindow._finish_close_if_ready(window)
        window.close.assert_not_called()
        importer.isRunning.return_value = False
        MainWindow._finish_close_if_ready(window)
        window.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
