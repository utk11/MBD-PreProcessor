"""JAX residual and Jacobian checks. JAX is required."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.kinematics import make_solver
from core.kinematics.backends.jax_cpu_backend import JaxCpuEvaluator
from core.kinematics.constraints import JointConstraint
from core.kinematics.engine import snapshot_state
from core.kinematics.markers import exp_so3, log_so3
from core.kinematics.prepared import prepare
from core.kinematics.workspace import create_workspace
from core.data_structures import JointType
from tests.kinematics_fixtures import (
    GROUND_POSE, Rz, build_four_bar, build_pendulum, build_single_joint, pose_of,
)


class JaxBackendTests(unittest.TestCase):
    def test_legacy_backend_is_rejected(self):
        bodies, joints, state = build_pendulum(False)
        with self.assertRaises(RuntimeError):
            make_solver(bodies, joints, state, backend="legacy")

    def test_all_joint_residuals_and_jacobians_match_constraints(self):
        for joint_type in JointType:
            self._check_joint_parity(joint_type, np.array([0.021, -0.013, 0.008]), Rz(7.0))

    def test_small_angle_and_near_pi_match_constraints(self):
        axis = np.array([0.2, -0.5, 0.8])
        axis = axis / np.linalg.norm(axis)
        small = exp_so3(1e-10 * axis)
        near_pi = exp_so3((np.pi - 1e-8) * axis)
        np.testing.assert_allclose(log_so3(small), 1e-10 * axis, atol=1e-14)
        np.testing.assert_allclose(log_so3(near_pi), (np.pi - 1e-8) * axis, atol=1e-8)
        for rotation in (small, near_pi):
            for joint_type in JointType:
                self._check_joint_parity(joint_type, np.zeros(3), rotation)

    def test_mechanisms_converge(self):
        for builder in (build_pendulum, build_four_bar):
            bodies, joints, state = builder(True)
            report = make_solver(bodies, joints, state).solve_assembly(max_iters=200, tol=1e-10)
            self.assertTrue(report.converged)
            self.assertLess(report.final_residual_norm, 1e-8)
            for body in bodies:
                pose = state.get_body_pose(body.id)
                self.assertTrue(np.isfinite(pose.origin).all())
                self.assertTrue(np.isfinite(pose.rotation_matrix).all())

    def _check_joint_parity(self, joint_type, shift, rotation):
        bodies, joints, state = build_single_joint(joint_type, axis="+Z", ground="parent")
        for body in bodies:
            pose = state.get_body_pose(body.id)
            state.set_body_pose(body.id, pose.origin + shift, rotation @ pose.rotation_matrix)
        model = prepare(bodies, joints)
        workspace = create_workspace(model)
        snapshot_state(state, model, workspace, GROUND_POSE)
        residual_jax = np.zeros_like(workspace.residual)
        blocks_jax = np.zeros_like(workspace.blocks)
        indices = np.arange(model.n_joints, dtype=np.int32)
        evaluator = JaxCpuEvaluator()
        count_jax = evaluator.evaluate(
            model, workspace.origin, workspace.rotation, indices,
            residual_jax, blocks_jax, True,
        )
        legacy = JointConstraint(
            joints[0], lambda body_id: pose_of(state, body_id),
            ground_pose=GROUND_POSE,
        )
        legacy_residual = legacy.residual()
        legacy_jacobian, movable_body_ids = legacy.jacobian()
        self.assertEqual(count_jax, legacy_residual.size)
        np.testing.assert_allclose(
            residual_jax[:count_jax], legacy_residual, atol=1e-10, rtol=1e-9
        )
        for column, body_id in enumerate(movable_body_ids):
            endpoint = 0 if joints[0].body1_id == body_id else 1
            width = legacy_residual.size
            np.testing.assert_allclose(
                blocks_jax[0, endpoint, :width, :],
                legacy_jacobian[:, 6 * column:6 * (column + 1)],
                atol=1e-9, rtol=1e-8,
            )
        sentinel = np.full_like(blocks_jax, 17.0)
        evaluator.evaluate(
            model, workspace.origin, workspace.rotation, indices,
            residual_jax, sentinel, False,
        )
        self.assertTrue(np.all(sentinel == 17.0))


if __name__ == "__main__":
    unittest.main()
