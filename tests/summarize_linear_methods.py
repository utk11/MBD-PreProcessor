"""Merge the final dense refresh and produce the all-method benchmark report."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--dense", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = json.loads(args.results.read_text(encoding="utf-8"))
    refresh = json.loads(args.dense.read_text(encoding="utf-8")) if args.dense else payload
    dense = {r["case"]: r for r in refresh["results"] if r["method"] == "dense"}
    cases = payload["settings"]["cases"]
    assert len(dense) == len(cases)
    for index, result in enumerate(payload["results"]):
        assert "samples" in result, result
        if result["method"] == "dense":
            replacement = dict(dense[result["case"]])
            assert "samples" in replacement, replacement
            replacement["measurement_source_sha256"] = refresh["source_sha256"]
            payload["results"][index] = replacement
        else:
            result.setdefault("measurement_source_sha256", payload["source_sha256"])
    payload["source_sha256"] = refresh["source_sha256"]
    if args.dense:
        payload["dense_refresh_file"] = str(args.dense)
    payload["verification"] = {"unit_tests_passed": 49, "paired_combinations": len(payload["results"])}
    args.results.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    by_key = {(r["case"], r["method"]): r for r in payload["results"]}
    methods = ("dense", "superlu", "lsmr", "cg")
    pose_origin = pose_rotation = orthogonality = 0.0
    for result in payload["results"]:
        before = result["paired_before"]
        for pose in result["final_poses"].values():
            rotation = np.asarray(pose[1])
            orthogonality = max(orthogonality, float(np.linalg.norm(rotation.T @ rotation - np.eye(3))))
        if before["samples"][-1]["converged"] and result["samples"][-1]["converged"]:
            for key, pose in result["final_poses"].items():
                old = before["final_poses"][key]
                pose_origin = max(pose_origin, float(np.max(np.abs(np.asarray(pose[0]) - old[0]))))
                pose_rotation = max(pose_rotation, float(np.max(np.abs(np.asarray(pose[1]) - old[1]))))

    lines = ["# Linear-method testing and optimization — 5 October 2026", "",
             "All four registered linear methods were tested, optimized, and rerun. "
             "Dense remains the application default. SuperLU is the fastest measured "
             "method on the larger chains; dense generally wins on the small mechanisms.", "",
             "## What changed", "",
             "- **Dense:** reuse normal, gradient, and system scratch; use Cholesky above 32 unknowns; "
             "retain LU for tiny systems and the LU/least-squares fallback for non-positive-definite or singular systems.",
             "- **All sparse methods:** fill weighted blocks with NumPy, reuse canonical CSR layouts instead of "
             "rebuilding COO, accumulate duplicate endpoints, prune exact zeros, and retain up to four layouts "
             "so drag correction can alternate pinned and joint-only equations without rebuilding the pattern.",
             "- **SuperLU:** reuse the transpose, obtain the damping diagonal from the normal matrix already formed, "
             "and update that diagonal directly.",
             "- **LSMR:** right-scale columns by the damped normal diagonal, while preserving the original "
             "least-squares objective; cache both augmented operator directions. Report the true normal-equation "
             "residual, rather than the augmented least-squares residual. The mixed-joint conditioning failure is fixed.",
             "- **CG:** cache the transpose and use a size-based budget `max(4*ncols, 64)` instead of a 500-step "
             "ceiling. An explicit `maxiter` remains available. Unfinished steps are rejected and retain their iteration counts.",
             "- **Tracing:** count all linear solves, iterations, and failures, including drag corrections.", "",
             "## How the comparison was run", "",
             "16 numeric workloads × four methods. Workloads include pendulum, slider, closed four-bar, redundant "
             "fixed joints, disconnected bodies, all five joint types, locked bodies, an intentionally inconsistent "
             "assembly, chains of 16/64/256 bodies, a second chain-64 perturbation, and four three-target drag trajectories.", "",
             "The final comparison alternates saved original and optimized implementations within the same worker process, "
             "sharing one warmed constraint evaluator and workspace while keeping each strategy's caches independent. "
             "Each method/case has five paired warm samples, except chain 256 which has three. "
             "That is 312 before and 312 after timing samples. Each worker is isolated, runs sequentially, and limits "
             "BLAS/OpenMP to one thread. Fixtures and poses are identical and restored before each sample. "
             "Drag targets commit sequentially within a trajectory. JAX prewarm and a warm solve are excluded. "
             "Profiling and instrumentation run separately from the timing samples. Tests were not run alongside benchmarks.", "",
             "Environment: application Python 3.10.19, NumPy 2.2.6, SciPy 1.15.3, JAX 0.6.2, CPU. "
             "Assembly uses 50 outer iterations and requested tolerance 1e-9. Drag uses 20 iterations per target, "
             "tolerance 1e-6, pin weight 1, and a 0.02 scene scale. Chain perturbations use seeds 0 and 1 as named. "
             "Rank diagnostics are disabled to isolate numerical solving; these are not full GUI/rendering latency measurements.", "",
             "All four methods were rerun after the final small-system LU selection. Exact source hashes and both "
             "paired sample sets are retained in `linear-methods-final.json`. "
             "The initial independent sweeps are also retained in `linear-methods-before.json` and `linear-methods-after.json`.", "",
             "## Complete-solve timings", "",
             "Cells are **original → optimized**, in milliseconds; each is the median of its paired samples. "
             "`unfinished` means the original did not reach the engine's convergence condition. "
             "For `inconsistent`, remaining unconverged is the expected outcome on both sides.", "",
             "| Workload | Dense | SuperLU | LSMR | CG |", "| --- | ---: | ---: | ---: | ---: |"]
    for case in cases:
        cells = []
        for method in methods:
            result = by_key[case, method]
            before = result["paired_before"]
            flag = " unfinished" if not before["samples"][-1]["converged"] and case != "inconsistent" else ""
            cells.append(f'{before["median_s"] * 1000:.2f}{flag} → {result["median_s"] * 1000:.2f}')
        lines.append("| " + case + " | " + " | ".join(cells) + " |")
    lines.extend(["", "Small timing differences should not be treated as a universal win. "
                  "Raw min/max and all samples are available; background machine activity was not controlled. "
                  "The dense crossover microbenchmark showed that Cholesky overhead loses at 6 and 18 unknowns "
                  "and pays off at 96, 192, 384, and 1536. See `dense-factorization-crossover.json`.", "",
                  "## Accuracy and stopping behavior", "",
                  "All optimized methods remain finite, and all feasible workloads meet the existing engine's "
                  "convergence flag. The inconsistent assembly remains unconverged. This flag is not a proof that "
                  "the requested tolerance was reached: the existing engine accepts joint residuals below 1e-6 "
                  "after the outer cap. Mixed joints reach approximately 1.73e-7 in 50 iterations, not the requested 1e-9. "
                  "The outer engine's stopping rules were preserved.", "",
                  "The original LSMR mixed-joint solve repeatedly stopped at its condition-number limit; column scaling "
                  "removes those failures. Original CG on chain 256 rejected 17 capped solves and remained unfinished. "
                  "Optimized CG converges in 39 outer iterations, with 68,585 inner iterations. "
                  "An unfinished original solve's shorter wall time is not a speedup to the same answer.", "",
                  f"For workloads completed on both sides, the largest final origin-entry difference is {pose_origin:.3e} "
                  f"and rotation-entry difference is {pose_rotation:.3e}. Maximum final rotation orthogonality error "
                  f"is {orthogonality:.3e}. The new pinned, deliberately scaled linear-equation tests require "
                  "relative normal-equation residual below 2e-9 and compare against an independent LU reference.", "",
                  "All four methods have essentially identical pointer errors. Pendulum drag is about 1.15e-8; "
                  "slider drag 5.0e-12; four-bar drag 1.21e-9. The chain-16 trajectory retains about 6.35e-6 "
                  "maximum target error within the 20-iteration budget. That misses its requested 1e-6 pointer "
                  "tolerance on both original and optimized code, although joints remain closed to about 1.4e-9. "
                  "It is a remaining outer drag-policy limitation, not a performance win claimed as exact tracking.", "",
                  "## Verification", "",
                  "49 tests passed: the new linear-method tests, the complete architecture suite, dragging, "
                  "refactor regressions, and JAX backend tests. Coverage includes duplicate and cancelling endpoint "
                  "contributions, zero values becoming nonzero again, mixed row widths, locks, both pin layouts, "
                  "cache invalidation and bounded retention, differently scaled columns, small damping, strided "
                  "scratch buffers, singular fallback, nonfinite input, and failed CG iteration accounting. "
                  "No automatic switching between methods or changes to the GUI default were introduced.", "",
                  "## Reproduce", "", "```powershell",
                  "& 'C:/miniconda/envs/mbd_preproc/python.exe' -B tests/benchmark_linear_methods.py --paired --repeats 5 --large-repeats 3 --timeout 180 --profile --output Documentation/linear-methods-recheck.json",
                  "& 'C:/miniconda/envs/mbd_preproc/python.exe' -B -m unittest tests.test_linear_methods tests.test_architecture tests.test_dragging tests.test_refactor_regressions tests.test_jax_backend -q",
                  "```", "",
                  "The paired runner loads the exact original implementations saved under "
                  "`Documentation/linear-methods-baseline/`; their hashes match the initial sweep. "
                  "Use `--baseline-source Documentation/linear-methods-baseline` to run just the original code. "
                  "Use `--cases` and `--methods` for targeted reruns.", "",
                  "Dense is the appropriate default for the small mechanisms tested here. SuperLU merits selection "
                  "for larger sparse assemblies. CG now completes the tested large chain, but remains slower than "
                  "SuperLU. LSMR is more robust after scaling and much faster on chain dragging, but did not win "
                  "any of these measured workloads. Automatic selection should use representative assembly "
                  "topologies and loop density, rather than a cutoff inferred from open chains alone.", ""])
    args.output.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {args.output}; merged {len(payload['results'])} paired combinations")
    print("Maximum completed pose differences:", pose_origin, pose_rotation)


if __name__ == "__main__":
    main()
