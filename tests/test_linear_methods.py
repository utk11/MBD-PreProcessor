"""Equation parity, sparse cache invalidation, and iterative failure contracts."""
from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from core.kinematics.backends import assemble_blocks
from core.kinematics.linear import solve_dense_damped
from core.kinematics.linear_sparse import CgNormalStrategy, assemble_weighted_coo, make_linear_strategy
from core.kinematics.prepared import ComponentPlan


def plan_for(indices, widths, scatter, ncols):
    counts = np.asarray(widths, dtype=np.int32)[indices]
    return ComponentPlan(member_ids=tuple(range(ncols // 6)),
                         joint_indices=np.asarray(indices, dtype=np.int32), joint_names=(),
                         row_offset=np.r_[0, np.cumsum(counts)[:-1]].astype(np.int32),
                         row_counts=counts, n_residual_rows=int(sum(counts)),
                         movable_body_ids=tuple(range(ncols // 6)),
                         movable_slots=np.arange(ncols // 6, dtype=np.int32),
                         scatter_col=np.asarray(scatter, dtype=np.int32), ncols=ncols)


def dense_reference(jacobian, residual, damping):
    normal = jacobian.T @ jacobian
    normal.flat[::normal.shape[0] + 1] += damping * np.maximum(np.diag(normal).copy(), 1e-12)
    rhs = -(jacobian.T @ residual)
    return np.linalg.solve(normal, rhs), normal, rhs


class LinearMethodTests(unittest.TestCase):
    def test_sparse_pattern_reuse_tracks_values_duplicates_and_zero_crossings(self):
        model = SimpleNamespace(n_rows=np.array([6, 3, 6], dtype=np.int32))
        plan = plan_for([2, 1, 0], model.n_rows, [[0, 0], [-1, 6], [0, 6]], 12)
        rng = np.random.default_rng(741)
        pattern = None
        for iteration in range(4):
            blocks = rng.normal(size=(3, 2, 6, 6))
            # Contributions can cancel completely, then become nonzero later.
            if iteration == 0:
                blocks[2, 1] = -blocks[2, 0]
            if iteration == 2:
                blocks[0] = 0
            for pin_width in (0, 3, 6):
                # Patterns differ between pin layouts; reuse each for two fills.
                reused = pattern if pin_width == 0 else None
                for weight in (1.0, 1e3):
                    expected = np.zeros((plan.n_residual_rows + pin_width, plan.ncols))
                    assemble_blocks(plan, model, blocks, expected)
                    expected[:plan.n_residual_rows] *= weight
                    if pin_width:
                        signs = np.r_[np.ones(min(pin_width, 3)), -np.ones(max(pin_width - 3, 0))]
                        expected[plan.n_residual_rows + np.arange(pin_width), 6 + np.arange(pin_width)] = signs * 2
                    matrix, reused = assemble_weighted_coo(plan, model, blocks, weight,
                                                          plan.n_residual_rows, 6 if pin_width else None,
                                                          pin_width, 2.0, reused)
                    np.testing.assert_allclose(matrix.toarray(), expected, rtol=1e-14, atol=1e-14)
                if pin_width == 0:
                    pattern = reused

    def test_all_steps_solve_the_same_system_with_pins_and_small_damping(self):
        rng = np.random.default_rng(29)
        model = SimpleNamespace(n_rows=np.array([6, 3, 6], dtype=np.int32))
        plan = plan_for([0, 1, 2], model.n_rows, [[-1, 0], [0, 6], [6, 12]], 18)
        blocks = rng.normal(size=(3, 2, 6, 6))
        # Disparate column scales require preconditioning; free columns require damping.
        blocks[:, :, :, 0] *= 0.01
        blocks[:, :, :, 5] *= 20.0
        residual = rng.normal(size=plan.n_residual_rows + 6)
        matrix, _ = assemble_weighted_coo(plan, model, blocks, 1000, plan.n_residual_rows, 6, 6, 2.0)
        jacobian = matrix.toarray()
        for damping in (1e-3, 1e-7):
            expected, system, rhs = dense_reference(jacobian, residual, damping)
            for method in ("dense", "superlu", "lsmr", "cg"):
                with self.subTest(damping=damping, method=method):
                    delta = np.zeros(plan.ncols)
                    if method == "dense":
                        result = solve_dense_damped(jacobian, residual, damping,
                                                    np.zeros_like(system), np.zeros(plan.ncols),
                                                    delta, np.zeros_like(system))
                    else:
                        result = make_linear_strategy(method).solve_from_blocks(
                            plan, model, blocks, residual, damping, 1000,
                            plan.n_residual_rows, 6, 6, 2.0, delta)
                    self.assertTrue(result.finite, result.failure_reason)
                    relative_residual = np.linalg.norm(system @ delta - rhs) / np.linalg.norm(rhs)
                    self.assertLess(relative_residual, 2e-9)
                    np.testing.assert_allclose(delta, expected, rtol=3e-5, atol=3e-8)
                    self.assertAlmostEqual(result.linear_residual, np.linalg.norm(system @ delta - rhs),
                                           delta=1e-7 * max(np.linalg.norm(rhs), 1))

    def test_cg_cap_failure_clears_step_and_preserves_iteration_count(self):
        rng = np.random.default_rng(1)
        model = SimpleNamespace(n_rows=np.array([6], dtype=np.int32))
        plan = plan_for([0], model.n_rows, [[-1, 0]], 6)
        blocks = rng.normal(size=(1, 2, 6, 6))
        delta = np.full(6, 99.0)
        result = CgNormalStrategy(maxiter=1).solve_from_blocks(
            plan, model, blocks, np.arange(1, 7, dtype=float), 1e-3, 1.0, 6, None, 0, 0, delta)
        self.assertFalse(result.finite)
        self.assertEqual(result.iterations, 1)
        self.assertIn("info=1", result.failure_reason)
        np.testing.assert_array_equal(delta, np.zeros(6))

    def test_dense_scratch_slices_and_singular_fallback(self):
        rng = np.random.default_rng(9)
        jacobian = rng.normal(size=(12, 6))
        jacobian[:, -1] = 0
        residual = rng.normal(size=12)
        normal, system = np.full((18, 18), 777.0), np.full((18, 18), 888.0)
        gradient, delta = np.zeros(18), np.zeros(18)
        expected, formed, rhs = dense_reference(jacobian, residual, 1e-3)
        result = solve_dense_damped(jacobian, residual, 1e-3, normal, gradient, delta, system)
        self.assertTrue(result.finite)
        np.testing.assert_allclose(delta[:6], expected, atol=1e-12)
        np.testing.assert_allclose(system[:6, :6], formed)
        np.testing.assert_array_equal(normal[6:], 777.0)
        np.testing.assert_array_equal(system[6:], 888.0)
        singular = solve_dense_damped(jacobian, residual, 0, normal, gradient, delta, system)
        self.assertTrue(singular.finite)
        self.assertTrue(singular.used_lstsq)
        self.assertLess(np.linalg.norm(jacobian.T @ (jacobian @ delta[:6] + residual)), 1e-11)

    def test_sparse_strategy_invalidates_pattern_when_scatter_or_pins_change(self):
        model = SimpleNamespace(n_rows=np.array([6], dtype=np.int32))
        initial = plan_for([0], model.n_rows, [[-1, 0]], 12)
        blocks = np.zeros((1, 2, 6, 6))
        blocks[0, 1] = np.eye(6)
        for method in ("superlu", "lsmr", "cg"):
            strategy = make_linear_strategy(method)
            for column, pin_width in ((0, 0), (6, 3), (0, 6), (6, 0)):
                plan = replace(initial, scatter_col=np.array([[-1, column]], dtype=np.int32))
                residual = np.ones(6 + pin_width)
                delta = np.zeros(12)
                result = strategy.solve_from_blocks(plan, model, blocks, residual, 1e-3,
                                                     1.0, 6, column, pin_width, 2.0, delta)
                self.assertTrue(result.finite, result.failure_reason)
                matrix, _ = assemble_weighted_coo(plan, model, blocks, 1.0, 6, column, pin_width, 2.0)
                expected, _, _ = dense_reference(matrix.toarray(), residual, 1e-3)
                np.testing.assert_allclose(delta, expected, rtol=1e-9, atol=1e-9)

    def test_trace_counts_failed_inner_iterations(self):
        from core.kinematics.solver import KinematicSolver
        from tests.kinematics_fixtures import build_pendulum
        bodies, joints, state = build_pendulum(True)
        solver = KinematicSolver(bodies, joints, state, linear_solver="cg")
        solver.linear_strategy = CgNormalStrategy(maxiter=1)
        try:
            report = solver.solve_assembly(max_iters=2, analyze=False, commit=False)
            self.assertTrue(report.finite)
            self.assertFalse(report.converged)
            self.assertEqual(report.trace.linear_solves, 2)
            self.assertEqual(report.trace.linear_failures, 2)
            self.assertEqual(report.trace.linear_iterations, 2)
        finally:
            solver.release()

    def test_dense_nonfinite_input_is_a_failed_step(self):
        for value in (np.nan, np.inf):
            jacobian = np.eye(6)
            jacobian[0, 0] = value
            with np.errstate(invalid="ignore"):
                result = solve_dense_damped(jacobian, np.ones(6), 1e-3,
                                            np.zeros((6, 6)), np.zeros(6), np.zeros(6), np.zeros((6, 6)))
            self.assertFalse(result.finite)

    def test_large_dense_cholesky_matches_independent_lu_with_strided_scratch(self):
        rng = np.random.default_rng(17)
        jacobian = rng.normal(size=(104, 96))
        jacobian[:, -1] = 0
        residual = rng.normal(size=104)
        expected, system, rhs = dense_reference(jacobian, residual, 1e-3)
        delta = np.zeros(120)
        result = solve_dense_damped(jacobian, residual, 1e-3, np.zeros((120, 120)),
                                    np.zeros(120), delta, np.zeros((120, 120)))
        self.assertEqual(result.factorization, "dense-cholesky")
        self.assertTrue(result.finite)
        np.testing.assert_allclose(delta[:96], expected, rtol=1e-10, atol=1e-12)
        self.assertLess(np.linalg.norm(system @ delta[:96] - rhs) / np.linalg.norm(rhs), 1e-12)

    def test_drag_layouts_are_cached_and_the_cache_is_bounded(self):
        from core.kinematics import linear_sparse
        model = SimpleNamespace(n_rows=np.array([6], dtype=np.int32))
        plan = plan_for([0], model.n_rows, [[-1, 0]], 12)
        blocks = np.zeros((1, 2, 6, 6))
        blocks[0, 1] = np.eye(6)
        for method in ("superlu", "lsmr", "cg"):
            strategy = make_linear_strategy(method)
            with patch.object(linear_sparse, "_build_pattern", wraps=linear_sparse._build_pattern) as build:
                for width in (3, 0, 3, 0, 3, 0):
                    result = strategy.solve_from_blocks(plan, model, blocks, np.ones(6 + width),
                                                         1e-3, 1, 6, 0 if width else None,
                                                         width, 1, np.zeros(12))
                    self.assertTrue(result.finite)
                self.assertEqual(build.call_count, 2)
            for column in range(7):
                strategy.solve_from_blocks(plan, model, blocks, np.ones(9), 1e-3,
                                            1, 6, column, 3, 1, np.zeros(12))
            self.assertLessEqual(len(strategy._patterns), 4)


if __name__ == "__main__":
    unittest.main()
