"""Document, queue, session-cache, and linear-step checks."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.assembly_document import AssemblyDocument
from core.data_structures import Force, Frame, Joint, JointType, MotorType, Torque
from core.kinematics import KinematicSolver
from core.kinematics.backends import assemble_blocks
from core.kinematics.engine import snapshot_state
from core.kinematics.linear import solve_dense_damped
from core.kinematics.linear_sparse import assemble_weighted_coo, make_linear_strategy
from core.kinematics.prepared import ComponentPlan, prepare
from core.kinematics.reports import SolveReport, commit_poses
from core.kinematics.solver import SolveRequest
from core.kinematics.reports import RevisionStamp
from core.kinematics.workspace import create_workspace
from core.project_store import (
    apply_project,
    read_project,
    save_project,
    validate_against_bodies,
    ProjectValidationError,
)
from gui.solve_scheduler import SolveQueue, result_is_acceptable
from tests.kinematics_fixtures import (
    GROUND_POSE,
    build_four_bar,
    build_overconstrained,
    build_pendulum,
    make_body,
    make_state,
    pose_of,
)


def _report(**kwargs):
    report = SolveReport(True, 1, 0.0, 0.0, finite=True, **kwargs)
    return report


class SessionTests(unittest.TestCase):
    def test_marker_edit_reuses_compiled_kernels(self):
        bodies, joints, state = build_pendulum(True)
        solver = KinematicSolver(bodies, joints, state)
        solver.solve_assembly(max_iters=5, tol=1e-8, analyze=False)
        compiled = solver.evaluator.compile_count
        self.assertGreater(compiled, 0)
        joints[0].marker1.origin = joints[0].marker1.origin + np.array([0.01, 0.0, 0.0])
        solver.solve_assembly(max_iters=5, tol=1e-8, analyze=False)
        self.assertEqual(solver.evaluator.compile_count, compiled)

        solver.evaluator.cache_cap = 1
        bodies2, joints2, state2 = build_four_bar(True)
        solver.bodies = bodies2
        solver.joints = joints2
        solver.state = state2
        solver.invalidate("topology")
        solver.solve_assembly(max_iters=2, tol=1e-8, analyze=False)
        self.assertEqual(solver.evaluator.compile_count, compiled + 2)
        self.assertEqual(len(solver.evaluator._compiled), 1)

    def test_drag_skips_rank_unless_asked(self):
        bodies, joints, state = build_pendulum(True)
        solver = KinematicSolver(bodies, joints, state)
        origin, rotation = pose_of(state, 1)
        report = solver.solve_drag(
            1, origin + np.array([0.05, 0.02, 0.0]), rotation,
            pin_weight=1.0, max_iters=12, tol=1e-6, pin_orientation=False,
        )
        self.assertIsNone(report.dof)
        analyzed = solver.solve_drag(
            1, origin + np.array([0.08, 0.01, 0.0]), rotation,
            pin_weight=1.0, max_iters=12, tol=1e-6, pin_orientation=False,
            analyze=True,
        )
        self.assertIsNotNone(analyzed.dof)

    def test_nonfinite_pose_is_not_committed(self):
        bodies, joints, state = build_pendulum(False)
        kept = pose_of(state, 1)
        solver = KinematicSolver(bodies, joints, state)
        request = SolveRequest(
            kind="assembly",
            bodies=bodies,
            joints=joints,
            poses={1: (np.array([np.nan, 0.0, 0.0]), np.eye(3))},
            ground_pose=GROUND_POSE,
            stamp=RevisionStamp(kind="assembly"),
            max_iters=4,
            tol=1e-9,
            analyze=False,
        )
        report = solver.solve_request(request)
        self.assertFalse(report.finite)
        self.assertEqual(report.poses, {})
        origin, rotation = pose_of(state, 1)
        np.testing.assert_allclose(origin, kept[0])
        np.testing.assert_allclose(rotation, kept[1])
        poisoned = SolveReport(
            False, 1, 1.0, 1.0, finite=False,
            poses={1: (np.ones(3), np.eye(3))},
        )
        commit_poses(state, poisoned)
        origin, _rotation = pose_of(state, 1)
        np.testing.assert_allclose(origin, kept[0])


class DocumentTests(unittest.TestCase):
    def _scene(self):
        document = AssemblyDocument()
        first = make_body(1, [0.0, 0.0, 0.0], name="first")
        second = make_body(2, [1.0, 0.0, 0.0], name="second")
        document.replace_bodies([first, second])
        document.state = make_state([first, second])
        frame = Frame(origin=np.array([0.2, 0.0, 0.0]), name="user")
        document.add_frame(frame, 1, "reference_geometry")
        joint = Joint("hinge", JointType.REVOLUTE, 1, 2, Frame(name="joint"), "+Z")
        document.add_joint(joint)
        document.add_force(Force("push", 1, Frame(name="force"), 3.0, np.array([1.0, 0.0, 0.0])))
        document.add_torque(Torque("twist", 2, Frame(name="torque"), 1.5, np.array([0.0, 0.0, 1.0])))
        return document

    def test_delete_one_and_many_bodies(self):
        document = self._scene()
        change = document.delete_bodies([1])
        self.assertEqual([body.id for body in document.bodies], [2])
        self.assertEqual(change.deleted_joints, ["hinge"])
        self.assertEqual(change.deleted_frames, ["user"])
        self.assertEqual(change.deleted_forces, ["push"])
        self.assertNotIn("hinge", document.joints)
        self.assertNotIn("user", document.frames)
        self.assertIsNone(document.state.get_body_pose(1))

        document = self._scene()
        change = document.delete_bodies([1, 2])
        self.assertEqual(document.bodies, [])
        self.assertEqual(change.deleted_torques, ["twist"])
        self.assertEqual(document.joints, {})
        self.assertEqual(document.forces, {})
        self.assertEqual(document.torques, {})


class ProjectTests(unittest.TestCase):
    def test_v2_round_trip_and_v1_migration(self):
        document = AssemblyDocument()
        body = make_body(1, [0.0, 0.0, 0.0])
        document.replace_bodies([body])
        document.state = make_state([body])
        document.state.set_body_pose(1, np.array([0.3, -0.1, 0.2]), np.eye(3))
        document.add_frame(Frame(origin=np.array([0.1, 0.0, 0.0]), name="face"), 1, "reference_geometry")
        joint = Joint("hinge", JointType.REVOLUTE, -1, 1, Frame(name="jf"), "+Z")
        joint.marker1 = Frame(origin=np.array([0.0, 0.0, 0.0]), name="m1")
        joint.marker2 = Frame(origin=np.array([0.0, 0.0, 0.2]), name="m2")
        joint.add_motor(MotorType.VELOCITY, 1.25)
        document.add_joint(joint)
        document.add_force(Force("push", 1, Frame(name="ff"), 4.0, np.array([0.0, 1.0, 0.0])))
        with tempfile.TemporaryDirectory() as folder:
            step = os.path.join(folder, "part.step")
            with open(step, "wb") as handle:
                handle.write(b"step-bytes")
            path = os.path.join(folder, "model.mbdp")
            save_project(path, document, step)
            self.assertFalse(any(name.endswith(".tmp") for name in os.listdir(folder)))
            loaded = read_project(path)
            self.assertEqual(loaded.schema_version, 2)
            validate_against_bodies(loaded, [1])
            restored = AssemblyDocument()
            restored_body = make_body(1, [0.0, 0.0, 0.0])
            restored.replace_bodies([restored_body])
            restored.state = make_state([restored_body])
            apply_project(restored, loaded)
            origin, _rotation = pose_of(restored.state, 1)
            np.testing.assert_allclose(origin, [0.3, -0.1, 0.2])
            self.assertEqual(restored.frame_coordinates["face"], "reference_geometry")
            self.assertEqual(restored.frame_to_body["face"], 1)
            self.assertTrue(restored.joints["hinge"].is_motorized)
            np.testing.assert_allclose(restored.joints["hinge"].marker2.origin, [0.0, 0.0, 0.2])
            self.assertIn("push", restored.forces)

            v1_path = os.path.join(folder, "old.mbdp")
            payload = {
                "version": "1.0",
                "step_file": step,
                "unit_scale": 0.001,
                "frames": [{
                    "name": "worldish",
                    "origin": [1.0, 0.0, 0.0],
                    "rotation_matrix": np.eye(3).tolist(),
                }],
                "joints": [{
                    "name": "old",
                    "type": "REVOLUTE",
                    "body1_id": -1,
                    "body2_id": 1,
                    "frame_name": "jf",
                    "frame_origin": [0.0, 0.0, 0.0],
                    "frame_rotation": np.eye(3).tolist(),
                    "axis": "+Z",
                }],
            }
            with open(v1_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle)
            old = read_project(v1_path)
            self.assertEqual(old.schema_version, 1)
            self.assertGreaterEqual(len(old.notes), 4)
            fresh = AssemblyDocument()
            fresh_body = make_body(1, [0.4, 0.0, 0.0])
            fresh.replace_bodies([fresh_body])
            fresh.state = make_state([fresh_body])
            notes = apply_project(fresh, old)
            self.assertTrue(any("version 1" in note for note in notes))
            origin, _rotation = pose_of(fresh.state, 1)
            np.testing.assert_allclose(origin, [0.4, 0.0, 0.0])
            self.assertIsNone(fresh.frame_to_body.get("worldish"))
            self.assertEqual(fresh.frame_coordinates["worldish"], "world")
            self.assertIsNotNone(fresh.joints["old"].marker1)
            self.assertIsNotNone(fresh.joints["old"].marker2)

    def test_unknown_body_reference_is_rejected(self):
        payload = {
            "version": "1.0",
            "step_file": "missing.step",
            "frames": [],
            "joints": [{
                "name": "bad",
                "type": "FIXED",
                "body1_id": -1,
                "body2_id": 99,
            }],
        }
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "bad.mbdp")
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle)
            loaded = read_project(path)
            with self.assertRaises(ProjectValidationError):
                validate_against_bodies(loaded, [1])


class QueueTests(unittest.TestCase):
    def test_one_pending_target_and_final_is_kept(self):
        queue = SolveQueue()
        queue.begin_gesture()
        queue.submit_drag(1, np.zeros(3))
        latest = queue.submit_drag(1, np.array([0.2, 0.0, 0.0]))
        self.assertIs(queue.pending, latest)
        self.assertEqual(queue.pending.request_id, 2)
        queue.mark_final()
        self.assertTrue(queue.pending.finalize)
        dispatched = queue.pop_for_dispatch()
        self.assertTrue(dispatched.finalize)
        self.assertIsNone(queue.pending)

    def test_prewarm_does_not_replace_a_pending_drag(self):
        queue = SolveQueue()
        queue.begin_gesture()
        queue.submit_drag(1, np.ones(3))
        queue.request_prewarm()
        first = queue.pop_for_dispatch()
        self.assertEqual(first.kind, "prewarm")
        self.assertEqual(queue.pending.kind, "drag")
        queue.active = None
        second = queue.pop_for_dispatch()
        self.assertEqual(second.kind, "drag")

    def test_epoch_rejects_and_intermediate_drag_can_commit(self):
        queue = SolveQueue()
        queue.begin_gesture()
        drag = queue.submit_drag(1, np.zeros(3))
        older = _report(
            document_generation=1,
            topology_revision=2,
            marker_revision=3,
            external_pose_epoch=4,
            pose_revision=5,
            request_id=drag.request_id,
            gesture_id=drag.gesture_id,
            epoch=drag.epoch,
            kind="drag",
        )
        self.assertTrue(result_is_acceptable(
            older, generation=1, topology=2, marker=3, external=4, pose=5,
            epoch=drag.epoch, gesture=drag.gesture_id, last_committed=0,
        ))
        queue.submit_assembly()
        self.assertFalse(result_is_acceptable(
            older, generation=1, topology=2, marker=3, external=4, pose=5,
            epoch=queue.epoch, gesture=drag.gesture_id, last_committed=0,
        ))
        queue.note_accepted(drag.request_id)
        self.assertFalse(result_is_acceptable(
            older, generation=1, topology=2, marker=3, external=4, pose=9,
            epoch=drag.epoch, gesture=drag.gesture_id, last_committed=queue.last_committed_request_id,
        ))


class SparseLinearTests(unittest.TestCase):
    def test_pattern_accumulates_duplicates_and_skips_locked_columns(self):
        plan = ComponentPlan(
            member_ids=(1,),
            joint_indices=np.array([0], dtype=np.int32),
            joint_names=("j",),
            row_offset=np.array([0], dtype=np.int32),
            row_counts=np.array([6], dtype=np.int32),
            n_residual_rows=6,
            movable_body_ids=(1,),
            movable_slots=np.array([1], dtype=np.int32),
            scatter_col=np.array([[0, 0]], dtype=np.int32),
            ncols=6,
        )

        class _Model:
            n_rows = np.array([6], dtype=np.int32)

        blocks = np.zeros((1, 2, 6, 6))
        blocks[0, 0] = np.eye(6)
        blocks[0, 1] = 2.0 * np.eye(6)
        matrix, _pattern = assemble_weighted_coo(
            plan, _Model(), blocks, 1.0, 6, None, 0, 0.0,
        )
        np.testing.assert_allclose(matrix.toarray(), 3.0 * np.eye(6))

        locked = ComponentPlan(
            member_ids=(1,),
            joint_indices=np.array([0], dtype=np.int32),
            joint_names=("ground",),
            row_offset=np.array([0], dtype=np.int32),
            row_counts=np.array([6], dtype=np.int32),
            n_residual_rows=9,
            movable_body_ids=(1,),
            movable_slots=np.array([1], dtype=np.int32),
            scatter_col=np.array([[-1, 0]], dtype=np.int32),
            ncols=6,
        )
        pinned, _pattern = assemble_weighted_coo(
            locked, _Model(), blocks, 1.0, 6, 0, 3, 2.0,
        )
        dense = np.zeros((9, 6))
        dense[:6, :] = 2.0 * np.eye(6)
        dense[6, 0] = 2.0
        dense[7, 1] = 2.0
        dense[8, 2] = 2.0
        np.testing.assert_allclose(pinned.toarray(), dense)

    def test_sparse_steps_match_dense(self):
        timings = {}
        for name, builder in (
            ("pendulum", build_pendulum),
            ("four_bar", build_four_bar),
            ("redundant", build_overconstrained),
        ):
            dense_step, seconds = _one_linear_step(builder, "dense")
            timings[name] = {"dense": seconds}
            for strategy in ("superlu", "lsmr", "cg"):
                other, seconds = _one_linear_step(builder, strategy)
                timings[name][strategy] = seconds
                np.testing.assert_allclose(other, dense_step, atol=1e-8, rtol=1e-8)
        print("SPARSE_TIMING", json.dumps(timings))

    def test_cg_assembly_matches_dense(self):
        for name, builder in (("pendulum", build_pendulum), ("four_bar", build_four_bar)):
            with self.subTest(name=name):
                dense_bodies, dense_joints, dense_state = builder(True)
                cg_bodies, cg_joints, cg_state = builder(True)
                dense = KinematicSolver(
                    dense_bodies, dense_joints, dense_state, diagnostics="off", linear_solver="dense",
                )
                cg = KinematicSolver(
                    cg_bodies, cg_joints, cg_state, diagnostics="off", linear_solver="cg",
                )
                try:
                    dense_report = dense.solve_assembly(max_iters=50, tol=1e-9, analyze=False)
                    cg_report = cg.solve_assembly(max_iters=50, tol=1e-9, analyze=False)
                    self.assertTrue(dense_report.finite and cg_report.finite)
                    self.assertEqual(dense_report.iterations, cg_report.iterations)
                    self.assertEqual(dense_report.converged, cg_report.converged)
                    self.assertEqual(dense_report.converged, True)
                    for body in dense_bodies:
                        dense_pose = dense_state.get_body_pose(body.id)
                        cg_pose = cg_state.get_body_pose(body.id)
                        np.testing.assert_allclose(cg_pose.origin, dense_pose.origin, atol=1e-6, rtol=1e-6)
                        np.testing.assert_allclose(
                            cg_pose.rotation_matrix, dense_pose.rotation_matrix, atol=1e-6, rtol=1e-6,
                        )
                finally:
                    dense.release()
                    cg.release()


def _one_linear_step(builder, strategy_name):
    bodies, joints, state = builder(True) if builder is not build_overconstrained else builder()
    model = prepare(bodies, joints)
    workspace = create_workspace(model, dense_linear=strategy_name == "dense")
    snapshot_state(state, model, workspace, GROUND_POSE)
    from core.kinematics.backends.jax_cpu_backend import JaxCpuEvaluator
    evaluator = JaxCpuEvaluator()
    plan = model.components[0]
    evaluator.evaluate(
        model, workspace.origin, workspace.rotation, plan.joint_indices,
        workspace.residual, workspace.blocks, True,
    )
    width = int(plan.n_residual_rows)
    weighted = np.array(workspace.residual[:width] * 1e3, dtype=np.float64, copy=True)
    delta = np.zeros(plan.ncols, dtype=np.float64)
    if strategy_name == "dense":
        jacobian = assemble_blocks(plan, model, workspace.blocks, np.zeros((width, plan.ncols)))
        jacobian *= 1e3
        started = time.perf_counter()
        for _ in range(5):
            solve_dense_damped(
                jacobian, weighted, 1e-3,
                np.zeros((plan.ncols, plan.ncols)),
                np.zeros(plan.ncols),
                delta,
                np.zeros((plan.ncols, plan.ncols)),
            )
        elapsed = (time.perf_counter() - started) / 5.0
        return delta.copy(), elapsed
    strategy = make_linear_strategy(strategy_name)
    started = time.perf_counter()
    last = None
    for _ in range(5):
        last = strategy.solve_from_blocks(
            plan, model, workspace.blocks, weighted, 1e-3, 1e3,
            width, None, 0, 0.0, delta,
        )
    elapsed = (time.perf_counter() - started) / 5.0
    if last is not None and not last.finite:
        raise AssertionError(f"{strategy_name} failed: {last.failure_reason}")
    return delta.copy(), elapsed


if __name__ == "__main__":
    unittest.main()
