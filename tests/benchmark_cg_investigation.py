"""Reproduce CG performance findings without changing production strategies.

Run with the application's Python, e.g.:
  python -B tests/benchmark_cg_investigation.py --output Documentation/cg-results.json
All assembly samples start from the same perturbed poses, exclude prewarm,
disable diagnostics, and use commit=False. Experimental variants are local.
"""
from __future__ import annotations

import argparse
import cProfile
import io
import json
import os
from pathlib import Path
import platform
import pstats
import statistics
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import jax
import numpy as np
import scipy
import scipy.sparse.linalg as spla

from core.kinematics import engine
from core.kinematics.linear import DIAG_FLOOR, StepResult
from core.kinematics import linear_sparse as sparse
from core.kinematics.solver import KinematicSolver
from tests.kinematics_fixtures import build_four_bar, build_open_chain, build_pendulum


def vector_fill(plan, model, blocks, joint_weight, pin_column, pin_width, pin_sqrt):
    chunks = []
    for local, joint_index in enumerate(plan.joint_indices):
        width = int(model.n_rows[joint_index])
        for endpoint in (0, 1):
            if plan.scatter_col[local, endpoint] >= 0:
                chunks.append(blocks[joint_index, endpoint, :width, :].ravel() * joint_weight)
    if pin_width and pin_column is not None:
        chunks.append(np.concatenate((np.ones(min(pin_width, 3)), -np.ones(max(pin_width - 3, 0)))) * pin_sqrt)
    return np.concatenate(chunks) if chunks else np.zeros(0)


class FastCG(sparse.CgNormalStrategy):
    """Experimental: vectorized fill, remove explicit zeros, cache transpose."""

    def __init__(self, transpose_only=False):
        super().__init__()
        self.transpose_only = transpose_only

    def solve_from_blocks(self, plan, model, blocks, residual, damping, joint_weight,
                          n_joint_rows, pin_column, pin_width, pin_sqrt, delta_out):
        key = sparse._pattern_key(plan, model, n_joint_rows, pin_column, pin_width)
        if key != self._key:
            self._pattern = sparse._build_pattern(plan, model, n_joint_rows, pin_column, pin_width)
            self._key = key
        with patch.object(sparse, "_fill_data", sparse._fill_data if self.transpose_only else vector_fill):
            jacobian, self._pattern = sparse.assemble_weighted_coo(
                plan, model, blocks, joint_weight, n_joint_rows,
                pin_column, pin_width, pin_sqrt, self._pattern,
            )
        if not self.transpose_only:
            jacobian.eliminate_zeros()
        transpose = jacobian.T
        self.jacobian_nnz = int(jacobian.nnz)
        ncols = plan.ncols
        if not ncols:
            return StepResult(0.0, False, True)
        column_sum = np.asarray(jacobian.multiply(jacobian).sum(axis=0)).ravel()
        diagonal = np.maximum(column_sum, DIAG_FLOOR)
        damp = damping * diagonal
        rhs = -(transpose @ residual)

        def matvec(v):
            return transpose @ (jacobian @ v) + damp * v

        operator = spla.LinearOperator((ncols, ncols), matvec=matvec, dtype=np.float64)
        scale = 1.0 / np.maximum(column_sum + damp, 1e-30)
        preconditioner = spla.LinearOperator((ncols, ncols), matvec=lambda v: scale * v, dtype=np.float64)
        iterations = [0]

        def counted(_):
            iterations[0] += 1

        solved, info = spla.cg(operator, rhs, M=preconditioner, rtol=1e-10,
                               atol=1e-12, maxiter=min(4 * ncols, 500), callback=counted)
        if info or not np.isfinite(solved).all():
            return sparse._failed(f"CG stopped with info={info}", delta_out, ncols)
        delta_out[:ncols] = solved
        return StepResult(float(np.linalg.norm(solved)), False, True,
                          float(np.linalg.norm(matvec(solved) - rhs)), iterations[0], "cg-fast")


def fixture(name):
    if name == "pendulum":
        return build_pendulum(True)
    if name == "four_bar":
        return build_four_bar(True)
    return build_open_chain(int(name.removeprefix("chain")), perturbed=True, seed=0)


def run_case(name, variant, repeats, profile):
    bodies, joints, state = fixture(name)
    strategy_name = variant if variant in ("dense", "superlu") else "cg"
    solver = KinematicSolver(bodies, joints, state, diagnostics="off", linear_solver=strategy_name)
    if "fast" in variant or variant == "cg-transpose":
        solver.linear_strategy = FastCG(transpose_only=variant == "cg-transpose")
    solver.prewarm()
    cg_original = spla.cg
    dense_original = engine.solve_dense_damped
    samples, calls, inner = [], [], []

    def cg_spy(a, b, **kwargs):
        if "cap" in variant:
            kwargs["maxiter"] = 4 * a.shape[0]
        callback = kwargs.get("callback")
        count = [0]

        def counted(x):
            count[0] += 1
            if callback is not None:
                callback(x)

        kwargs["callback"] = counted
        start = time.perf_counter()
        x, info = cg_original(a, b, **kwargs)
        inner.append({"iterations": count[0], "info": int(info),
                      "seconds": time.perf_counter() - start, "maxiter": kwargs["maxiter"]})
        return x, info

    def step_spy(original, *args, **kwargs):
        damping = float(args[4] if strategy_name != "dense" else args[2])
        start = time.perf_counter()
        result = original(*args, **kwargs)
        calls.append({"damping": damping, "seconds": time.perf_counter() - start,
                      "finite": result.finite, "failure_reason": result.failure_reason,
                      "linear_residual": result.linear_residual,
                      "iterations": result.iterations})
        return result

    def dense_spy(*args, **kwargs):
        return step_spy(dense_original, *args, **kwargs)

    block_original = getattr(solver.linear_strategy, "solve_from_blocks", None)

    def block_spy(*args, **kwargs):
        return step_spy(block_original, *args, **kwargs)

    step_patch = (patch.object(engine, "solve_dense_damped", dense_spy) if strategy_name == "dense"
                  else patch.object(solver.linear_strategy, "solve_from_blocks", block_spy))
    try:
        # Untimed first solve warms the path; commit=False preserves the fixture.
        solver.solve_assembly(max_iters=50, tol=1e-9, analyze=False, commit=False)
        with patch.object(spla, "cg", cg_spy), step_patch:
            for _ in range(repeats):
                calls.clear()
                inner.clear()
                start = time.perf_counter()
                report = solver.solve_assembly(max_iters=50, tol=1e-9, analyze=False,
                                               trace=True, commit=False)
                samples.append({"seconds": time.perf_counter() - start,
                                "outer_iterations": report.iterations, "converged": report.converged,
                                "finite": report.finite, "residual": report.final_residual_norm,
                                "max_residual": report.max_residual,
                                "phases_s": report.trace.phases_s,
                                "accepted": report.trace.accepted_steps,
                                "rejected": report.trace.rejected_steps,
                                "linear_calls": list(calls), "cg_calls": list(inner)})
        result = {"case": name, "variant": variant, "samples": samples,
                  "median_s": statistics.median(s["seconds"] for s in samples),
                  "final_poses": {str(k): [v[0].tolist(), v[1].tolist()] for k, v in report.poses.items()}}
        if profile and variant == "cg":
            profiler = cProfile.Profile()
            profiler.runcall(solver.solve_assembly, max_iters=50, tol=1e-9,
                             analyze=False, commit=False)
            stream = io.StringIO()
            pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(35)
            result["profile"] = stream.getvalue()
        return result
    finally:
        solver.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", default=["pendulum", "four_bar", "chain16", "chain64", "chain256"])
    parser.add_argument("--variants", nargs="+",
                        choices=["dense", "cg", "cg-transpose", "cg-fast", "cg-cap", "cg-fast-cap", "superlu"],
                        default=["dense", "cg", "cg-transpose", "cg-fast", "cg-cap", "cg-fast-cap", "superlu"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--profile", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = {"environment": {"python": sys.version, "executable": sys.executable,
                               "numpy": np.__version__, "scipy": scipy.__version__, "jax": jax.__version__,
                               "platform": platform.platform(),
                               "threads": {k: os.environ.get(k) for k in
                                           ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")}},
               "settings": {"repeats": args.repeats, "outer_maxiter": 50, "tol": 1e-9,
                            "diagnostics": False, "prewarmed": True, "commit": False}, "results": []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for name in args.cases:
        for variant in args.variants:
            result = run_case(name, variant, args.repeats, args.profile)
            payload["results"].append(result)
            args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            last = result["samples"][-1]
            print(name, variant, f'{1000 * result["median_s"]:.3f} ms',
                  "outer", last["outer_iterations"], "converged", last["converged"],
                  "inner", sum(c["iterations"] for c in last["cg_calls"]),
                  "failed", sum(c["info"] != 0 for c in last["cg_calls"]),
                  "residual", last["residual"], flush=True)


if __name__ == "__main__":
    main()
