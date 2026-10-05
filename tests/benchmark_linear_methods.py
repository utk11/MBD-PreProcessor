"""Isolated, reproducible before/after benchmarks of the four linear methods.

The parent runs one worker per case/method with a deadline. Workers time warm
solves without profiling, then collect a separate instrumented diagnostic run.
Input poses are reset before every sample; drag targets commit sequentially.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
METHODS = ("dense", "superlu", "lsmr", "cg")
CASES = ("pendulum", "slider", "four_bar", "redundant", "disconnected", "mixed",
         "locked_chain16", "inconsistent", "chain16_s0", "chain64_s0", "chain64_s1",
         "chain256_s0", "pendulum_drag", "slider_drag", "four_bar_drag", "chain16_drag")


def worker(case, method, repeats, profile, baseline_source=None, paired=False):
    sys.path.insert(0, str(ROOT))
    import cProfile
    import io
    import pstats
    from unittest.mock import patch
    import jax
    import numpy as np
    import scipy
    import scipy.sparse.linalg as spla
    from core.data_structures import JointType
    from core.kinematics import engine
    from core.kinematics.solver import KinematicSolver
    from tests import kinematics_fixtures as f

    if baseline_source or paired:
        import importlib.util
        from importlib.machinery import SourceFileLoader
        from core.kinematics import solver as session

        baseline_directory = baseline_source or ROOT / "Documentation/linear-methods-baseline"

        def load_baseline(name):
            module_name = "_linear_benchmark_baseline_" + name
            loader = SourceFileLoader(module_name, str(baseline_directory / (name + ".py.txt")))
            spec = importlib.util.spec_from_loader(module_name, loader)
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            loader.exec_module(module)
            return module

        baseline_linear = load_baseline("linear")
        baseline_sparse = load_baseline("linear_sparse")
        if not paired:
            engine.solve_dense_damped = baseline_linear.solve_dense_damped
            session.make_linear_strategy = baseline_sparse.make_linear_strategy

    drag = case.endswith("_drag")
    base = case.removesuffix("_drag")
    locked = ()
    if base in ("pendulum", "slider", "four_bar"):
        bodies, joints, state = getattr(f, "build_" + base)(not drag)
    elif base == "redundant":
        bodies, joints, state = f.build_overconstrained()
        state.set_body_pose(1, [0.03, -0.02, 0.01], f.Rz(8))
    elif base == "disconnected":
        bodies, joints, state = f.build_two_pendulums(True)
    elif base == "inconsistent":
        bodies, joints, state = f.build_inconsistent_fixed()
    else:
        count = 6 if base == "mixed" else int(base.replace("locked_", "").split("_")[0][5:])
        seed = int(base.split("_s")[-1]) if "_s" in base else 0
        bodies, joints, state = f.build_open_chain(count, perturbed=not drag, seed=seed)
        if base == "mixed":
            for joint, kind in zip(joints, list(JointType) + [JointType.REVOLUTE]):
                joint.joint_type = kind
        if base.startswith("locked_"):
            locked = (1,)
            state.set_body_pose(1, [1, 0, 0], np.eye(3))

    targets = []
    dragged_id = 1
    if drag:
        scale = 0.02
        for body in bodies:
            body.local_frame.origin *= scale
            origin, rotation = f.pose_of(state, body.id)
            state.set_body_pose(body.id, origin * scale, rotation)
        for joint in joints:
            for frame in (joint.frame, joint.marker1, joint.marker2):
                frame.origin *= scale
        if base == "pendulum":
            targets = [scale * np.array([np.cos(np.radians(a)), np.sin(np.radians(a)), -1])
                       for a in (10, 20, -10)]
        elif base == "four_bar":
            targets = [scale * np.array([np.sin(np.radians(a)), 0, np.cos(np.radians(a))])
                       for a in (5, -5, 10)]
        elif base == "slider":
            targets = [np.array([0, 0, 0.01 + d]) for d in (0.001, -0.002, 0.003)]
        else:
            dragged_id = bodies[-1].id
            tip = f.pose_of(state, dragged_id)[0]
            # Exactly reachable targets from a common small rotation of the chain.
            targets = [f.Rz(a) @ tip for a in (0.1, -0.1, 0.2)]

    initial = f.copy_poses(state, [b.id for b in bodies])
    solver = KinematicSolver(bodies, joints, state, diagnostics="off", linear_solver=method,
                             locked_body_ids=locked)
    solver.prewarm()
    baseline_solver = None
    optimized_strategy = solver.linear_strategy
    if paired:
        # One compiled evaluator and workspace remove evaluator-instance timing
        # differences; strategies retain their own independent pattern caches.
        baseline_solver = solver
        baseline_strategy = baseline_sparse.make_linear_strategy(method)
    model = solver._model
    max_iters = 20 if drag else 50
    tol = 1e-6 if drag else 1e-9

    def solve(trace=False, baseline=False):
        f.restore_poses(state, initial)
        active = baseline_solver if baseline else solver
        if paired:
            solver.linear_strategy = baseline_strategy if baseline else optimized_strategy
        current_dense = engine.solve_dense_damped
        if baseline:
            engine.solve_dense_damped = baseline_linear.solve_dense_damped
        try:
            if not drag:
                return [active.solve_assembly(max_iters=max_iters, tol=tol, analyze=False,
                                              trace=trace, commit=False)]
            return [active.solve_drag(dragged_id, target, pin_weight=1.0, max_iters=max_iters,
                                      tol=tol, analyze=False, trace=trace, commit=True)
                    for target in targets]
        finally:
            engine.solve_dense_damped = current_dense

    def summary(reports):
        return {"converged": all(r.converged for r in reports),
                "finite": all(r.finite for r in reports),
                "outer_iterations": sum(r.iterations for r in reports),
                "max_residual": max(r.max_residual for r in reports),
                "residual_norm": max(r.final_residual_norm for r in reports),
                "target_error": max((r.target_error or 0) for r in reports),
                "reports": [{"converged": r.converged, "max_residual": r.max_residual,
                             "target_error": r.target_error} for r in reports]}

    try:
        solve()
        if paired:
            solve(baseline=True)
        samples, baseline_samples = [], []
        for repeat in range(repeats):
            order = ([False, True] if repeat % 2 == 0 else [True, False]) if paired else [False]
            for baseline in order:
                start = time.perf_counter()
                current_reports = solve(baseline=baseline)
                sample = {"seconds": time.perf_counter() - start, **summary(current_reports)}
                if baseline:
                    baseline_samples.append(sample)
                    baseline_poses = {str(k): [v[0].tolist(), v[1].tolist()]
                                      for k, v in current_reports[-1].poses.items()}
                else:
                    samples.append(sample)
                    reports = current_reports
        poses = {str(k): [v[0].tolist(), v[1].tolist()] for k, v in reports[-1].poses.items()}
        if paired:
            solver.linear_strategy = optimized_strategy
        # Instrumentation is deliberately outside the timing samples.
        steps, inner = [], []
        cg_original, lsmr_original = spla.cg, spla.lsmr
        dense_original = engine.solve_dense_damped
        block_original = getattr(solver.linear_strategy, "solve_from_blocks", None)

        def cg_spy(a, b, **kwargs):
            callback = kwargs.get("callback")
            count = [0]

            def counted(x):
                count[0] += 1
                if callback:
                    callback(x)

            kwargs["callback"] = counted
            x, info = cg_original(a, b, **kwargs)
            inner.append({"method": "cg", "iterations": count[0], "stop": int(info)})
            return x, info

        def lsmr_spy(*args, **kwargs):
            result = lsmr_original(*args, **kwargs)
            inner.append({"method": "lsmr", "iterations": int(result[2]), "stop": int(result[1])})
            return result

        def step_spy(original, *args, **kwargs):
            start = time.perf_counter()
            result = original(*args, **kwargs)
            steps.append({"seconds": time.perf_counter() - start,
                          "damping": float(args[2] if method == "dense" else args[4]),
                          "finite": result.finite, "iterations": result.iterations,
                          "label": result.factorization, "failure": result.failure_reason})
            return result

        dense_spy = lambda *a, **k: step_spy(dense_original, *a, **k)
        block_spy = lambda *a, **k: step_spy(block_original, *a, **k)
        step_patch = (patch.object(engine, "solve_dense_damped", dense_spy) if method == "dense"
                      else patch.object(solver.linear_strategy, "solve_from_blocks", block_spy))
        with patch.object(spla, "cg", cg_spy), patch.object(spla, "lsmr", lsmr_spy), step_patch:
            diagnostics = solve(trace=True)
        phases = {}
        for report in diagnostics:
            for name, value in report.trace.phases_s.items():
                phases[name] = phases.get(name, 0) + value
        result = {"case": case, "method": method, "samples": samples,
                  "median_s": statistics.median(s["seconds"] for s in samples),
                  "min_s": min(s["seconds"] for s in samples), "max_s": max(s["seconds"] for s in samples),
                  "final_poses": poses, "diagnostic": {**summary(diagnostics), "steps": steps,
                                                       "inner": inner, "phases_s": phases},
                  "unknowns": sum(p.ncols for p in model.components),
                  "components": len(model.components), "max_iters": max_iters, "tol": tol,
                  "workspace_bytes": sum(v.nbytes for v in vars(solver._workspace).values() if isinstance(v, np.ndarray)),
                  "versions": {"python": sys.version, "numpy": np.__version__,
                               "scipy": scipy.__version__, "jax": jax.__version__}}
        if paired:
            result["paired_before"] = {"samples": baseline_samples, "final_poses": baseline_poses,
                                       "median_s": statistics.median(s["seconds"] for s in baseline_samples)}
        if profile:
            profiler = cProfile.Profile()
            profiler.runcall(solve)
            stream = io.StringIO()
            pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(30)
            result["profile"] = stream.getvalue()
        return result
    finally:
        solver.release()
        if baseline_solver is not None and baseline_solver is not solver:
            baseline_solver.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--large-repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--profile", action="store_true")
    parser.add_argument("--baseline-source", type=Path, help="Run the saved pre-optimization implementations.")
    parser.add_argument("--paired", action="store_true", help="Alternate baseline and optimized samples in one worker.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        result = worker(args.cases[0], args.methods[0], args.repeats, args.profile, args.baseline_source, args.paired)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return
    payload = {"settings": vars(args) | {"output": str(args.output),
               "baseline_source": str(args.baseline_source) if args.baseline_source else None}, "results": [],
               "source_sha256": {name: hashlib.sha256((args.baseline_source / (Path(name).name + ".txt")
                                                       if args.baseline_source else ROOT / name).read_bytes()).hexdigest()
                                 for name in ("core/kinematics/linear.py", "core/kinematics/linear_sparse.py")},
               "executable": sys.executable, "threads": {k: "1" for k in
                     ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")}}
    if args.paired:
        directory = args.baseline_source or ROOT / "Documentation/linear-methods-baseline"
        payload["baseline_sha256"] = {name: hashlib.sha256((directory / (name + ".py.txt")).read_bytes()).hexdigest()
                                      for name in ("linear", "linear_sparse")}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    scratch = args.output.with_suffix(".worker.json")
    for case in args.cases:
        methods = list(args.methods)
        random.Random(42).shuffle(methods)
        for method in methods:
            repeats = args.large_repeats if "256" in case else args.repeats
            command = [sys.executable, "-B", str(Path(__file__).resolve()), "--worker",
                       "--cases", case, "--methods", method, "--repeats", str(repeats), "--output", str(scratch)]
            if args.profile:
                command.append("--profile")
            if args.baseline_source:
                command.extend(["--baseline-source", str(args.baseline_source)])
            if args.paired:
                command.append("--paired")
            try:
                completed = subprocess.run(command, cwd=ROOT, env=env, capture_output=True,
                                           text=True, timeout=args.timeout)
                if completed.returncode:
                    result = {"case": case, "method": method, "error": completed.stderr[-4000:]}
                else:
                    result = json.loads(scratch.read_text(encoding="utf-8"))
            except subprocess.TimeoutExpired:
                result = {"case": case, "method": method, "timeout_s": args.timeout}
            payload["results"].append(result)
            args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            if "samples" in result:
                d = result["diagnostic"]
                print(case, method, f'{result["median_s"] * 1000:.3f} ms',
                      "converged", d["converged"], "max_residual", f'{d["max_residual"]:.2e}',
                      "target", f'{d["target_error"]:.2e}',
                      "inner", sum(i["iterations"] for i in d["inner"]), flush=True)
            else:
                print(case, method, "TIMEOUT" if "timeout_s" in result else result["error"], flush=True)
    scratch.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
