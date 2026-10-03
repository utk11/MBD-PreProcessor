"""Camera mapping, final pointer targets, and responsive constrained dragging."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from PySide6.QtCore import QPoint, Qt

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from main import MainWindow
from core.kinematics import KinematicSolver
from gui.drag_projection import screen_drag_delta
from gui.viewer_3d import SelectableViewer3d
from tests.kinematics_fixtures import build_pendulum, build_four_bar


class OrthographicView:
    def __init__(self, units_per_pixel=1.0, right=(1, 0, 0), up=(0, 1, 0)):
        self.scale = units_per_pixel
        self.right = np.asarray(right, dtype=float)
        self.up = np.asarray(up, dtype=float)
        self.normal = np.cross(self.right, self.up)

    def Proj(self):
        return tuple(self.normal)

    def ConvertWithProj(self, x, y):
        point = self.scale * (x * self.right - y * self.up)
        return (*point, *(-self.normal))


class CameraDragTests(unittest.TestCase):
    def test_meters_millimeters_and_inches_have_the_same_pointer_displacement(self):
        for unit in (1.0, 0.001, 0.0254):
            with self.subTest(unit=unit):
                delta = screen_drag_delta(
                    OrthographicView(0.001 / unit), (100, 50), (110, 70), [0.0, 0.0, 0.1], unit,
                )
                np.testing.assert_allclose(delta, [0.01, -0.02, 0.0])

    def test_high_dpi_and_zoom_follow_camera_pixels(self):
        normal = screen_drag_delta(OrthographicView(1.0), (20, 20), (30, 40), [0, 0, 0], 0.001)
        high_dpi = screen_drag_delta(OrthographicView(0.5), (20, 20), (30, 40), [0, 0, 0], 0.001, 2.0)
        zoomed = screen_drag_delta(OrthographicView(0.25), (20, 20), (30, 40), [0, 0, 0], 0.001)
        np.testing.assert_allclose(normal, high_dpi)
        np.testing.assert_allclose(zoomed, normal / 4.0)

    def test_rotated_camera_and_off_center_click(self):
        view = OrthographicView(right=(0, 1, 0), up=(0, 0, 1))
        delta = screen_drag_delta(view, (100, 200), (110, 220), [5, 6, 7], 0.001)
        np.testing.assert_allclose(delta, [0.0, 0.01, -0.02])
        unchanged = screen_drag_delta(view, (100, 200), (100, 200), [5, 6, 7], 0.001)
        np.testing.assert_array_equal(unchanged, np.zeros(3))

    def test_perspective_uses_body_depth(self):
        view = SimpleNamespace(
            Proj=lambda: (0, 0, 1),
            ConvertWithProj=lambda x, y: (0, 0, 0, x / 100.0, -y / 100.0, 1),
        )
        near = screen_drag_delta(view, (10, 20), (15, 30), [0, 0, 2], 0.001)
        far = screen_drag_delta(view, (10, 20), (15, 30), [0, 0, 4], 0.001)
        np.testing.assert_allclose(near, [0.1, -0.2, 0.0])
        np.testing.assert_allclose(far, 2.0 * near)

    def test_invalid_camera_cannot_invent_a_drag_target(self):
        view = SimpleNamespace(Proj=lambda: (0, 0, 1), ConvertWithProj=lambda x, y: (0, 0, 0, 1, 0, 0))
        with self.assertRaises(ValueError):
            screen_drag_delta(view, (0, 0), (1, 1), [0, 0, 0], 0.001)


class PointerTests(unittest.TestCase):
    def test_one_pixel_motion_is_delivered(self):
        moved = Mock()
        viewer = SimpleNamespace(
            _dragging_body_id=1, _drag_start_screen=(0, 0), _last_drag_screen=(0, 0),
            _drag_start_world_pos=np.array([1.0, 2.0, 3.0]), on_body_drag_move=moved,
            _screen_delta_to_world_delta=Mock(return_value=np.array([1e-7, 0.0, 0.0])),
        )
        SelectableViewer3d._update_body_drag(viewer, 1, 0)
        moved.assert_called_once()
        np.testing.assert_allclose(moved.call_args.args[1], [1.0 + 1e-7, 2.0, 3.0], atol=1e-12)

    def test_mouse_release_delivers_position_before_finishing(self):
        calls = []
        viewer = SimpleNamespace(
            devicePixelRatioF=lambda: 1.0, _dragging_body_id=1,
            _update_body_drag=lambda x, y, force=False: calls.append(("move", x, y, force)),
            _end_body_drag=lambda: calls.append(("end",)),
        )
        event = SimpleNamespace(pos=lambda: QPoint(101, 202), button=lambda: Qt.LeftButton)
        SelectableViewer3d.mouseReleaseEvent(viewer, event)
        self.assertEqual(calls, [("move", 101, 202, True), ("end",)])

    def test_zoomed_sub_micrometer_target_is_not_filtered_out(self):
        before = np.array([1.0, 2.0, 3.0])
        after = before + np.array([1e-7, 0.0, 0.0])
        controller = Mock()
        window = SimpleNamespace(
            _pending_drag_body_id=1, _pending_drag_pos=after,
            _dragging_body_ref=SimpleNamespace(state=object()),
            _last_applied_drag_pos=before, controller=controller,
        )
        MainWindow._apply_pending_drag_update(window)
        controller.submit_drag_target.assert_called_once()
        np.testing.assert_array_equal(controller.submit_drag_target.call_args.args[1], after)
        # Repeated timer ticks do not resubmit an unchanged target.
        MainWindow._apply_pending_drag_update(window)
        controller.submit_drag_target.assert_called_once()


def scale_scene(bodies, joints, state, scale):
    for body in bodies:
        body.local_frame.origin *= scale
        pose = state.get_body_pose(body.id)
        state.set_body_pose(body.id, pose.origin * scale, pose.rotation_matrix)
    for joint in joints:
        for frame in (joint.frame, joint.marker1, joint.marker2):
            frame.origin *= scale


class ConstrainedDragTests(unittest.TestCase):
    def test_small_mechanism_follows_reachable_targets_in_one_request(self):
        for strategy in ("dense", "superlu", "lsmr"):
            with self.subTest(strategy=strategy):
                bodies, joints, state = build_pendulum(False)
                scale_scene(bodies, joints, state, 0.02)
                solver = KinematicSolver(bodies, joints, state, linear_solver=strategy)
                try:
                    for angle in (30, 10, -20):
                        radians = np.radians(angle)
                        target = np.array([0.02 * np.cos(radians), 0.02 * np.sin(radians), -0.02])
                        report = solver.solve_drag(1, target, pin_weight=1.0, max_iters=12, tol=1e-6, analyze=False)
                        self.assertTrue(report.finite)
                        self.assertLess(report.max_residual, 1e-6)
                        self.assertLess(report.target_error, 5e-6)
                finally:
                    solver.release()

    def test_small_four_bar_moves_without_opening_joints(self):
        bodies, joints, state = build_four_bar(False)
        scale_scene(bodies, joints, state, 0.02)
        solver = KinematicSolver(bodies, joints, state)
        try:
            before = state.get_body_pose(1).origin.copy()
            # The vertical crank initially moves horizontally around its hinge.
            target = before + np.array([0.004, 0.0, 0.0])
            report = solver.solve_drag(1, target, pin_weight=1.0, max_iters=12, tol=1e-6, analyze=False)
            self.assertTrue(report.finite)
            self.assertLess(report.max_residual, 1e-6)
            self.assertGreater(state.get_body_pose(1).origin[0] - before[0], 0.002)
            self.assertLess(report.target_error, 0.002)
        finally:
            solver.release()


if __name__ == "__main__":
    unittest.main()
