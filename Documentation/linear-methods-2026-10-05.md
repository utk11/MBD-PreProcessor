# Linear-method testing and optimization — 5 October 2026

All four registered linear methods were tested, optimized, and rerun. Dense remains the application default. SuperLU is the fastest measured method on the larger chains; dense generally wins on the small mechanisms.

## What changed

- **Dense:** reuse normal, gradient, and system scratch; use Cholesky above 32 unknowns; retain LU for tiny systems and the LU/least-squares fallback for non-positive-definite or singular systems.
- **All sparse methods:** fill weighted blocks with NumPy, reuse canonical CSR layouts instead of rebuilding COO, accumulate duplicate endpoints, prune exact zeros, and retain up to four layouts so drag correction can alternate pinned and joint-only equations without rebuilding the pattern.
- **SuperLU:** reuse the transpose, obtain the damping diagonal from the normal matrix already formed, and update that diagonal directly.
- **LSMR:** right-scale columns by the damped normal diagonal, while preserving the original least-squares objective; cache both augmented operator directions. Report the true normal-equation residual, rather than the augmented least-squares residual. The mixed-joint conditioning failure is fixed.
- **CG:** cache the transpose and use a size-based budget `max(4*ncols, 64)` instead of a 500-step ceiling. An explicit `maxiter` remains available. Unfinished steps are rejected and retain their iteration counts.
- **Tracing:** count all linear solves, iterations, and failures, including drag corrections.

## How the comparison was run

16 numeric workloads × four methods. Workloads include pendulum, slider, closed four-bar, redundant fixed joints, disconnected bodies, all five joint types, locked bodies, an intentionally inconsistent assembly, chains of 16/64/256 bodies, a second chain-64 perturbation, and four three-target drag trajectories.

The final comparison alternates saved original and optimized implementations within the same worker process, sharing one warmed constraint evaluator and workspace while keeping each strategy's caches independent. Each method/case has five paired warm samples, except chain 256 which has three. That is 312 before and 312 after timing samples. Each worker is isolated, runs sequentially, and limits BLAS/OpenMP to one thread. Fixtures and poses are identical and restored before each sample. Drag targets commit sequentially within a trajectory. JAX prewarm and a warm solve are excluded. Profiling and instrumentation run separately from the timing samples. Tests were not run alongside benchmarks.

Environment: application Python 3.10.19, NumPy 2.2.6, SciPy 1.15.3, JAX 0.6.2, CPU. Assembly uses 50 outer iterations and requested tolerance 1e-9. Drag uses 20 iterations per target, tolerance 1e-6, pin weight 1, and a 0.02 scene scale. Chain perturbations use seeds 0 and 1 as named. Rank diagnostics are disabled to isolate numerical solving; these are not full GUI/rendering latency measurements.

All four methods were rerun after the final small-system LU selection. Exact source hashes and both paired sample sets are retained in `linear-methods-final.json`. The initial independent sweeps are also retained in `linear-methods-before.json` and `linear-methods-after.json`.

## Complete-solve timings

Cells are **original → optimized**, in milliseconds; each is the median of its paired samples. `unfinished` means the original did not reach the engine's convergence condition. For `inconsistent`, remaining unconverged is the expected outcome on both sides.

| Workload | Dense | SuperLU | LSMR | CG |
| --- | ---: | ---: | ---: | ---: |
| pendulum | 1.08 → 1.12 | 4.06 → 2.52 | 5.45 → 6.03 | 4.11 → 3.23 |
| slider | 0.88 → 1.18 | 3.95 → 2.36 | 3.60 → 3.87 | 3.07 → 2.25 |
| four_bar | 2.64 → 3.28 | 6.03 → 4.02 | 9.27 → 8.60 | 8.03 → 5.95 |
| redundant | 2.03 → 2.26 | 4.00 → 3.06 | 5.53 → 5.85 | 3.65 → 2.89 |
| disconnected | 2.85 → 2.97 | 6.20 → 3.16 | 11.68 → 10.42 | 3.88 → 3.85 |
| mixed | 35.71 → 32.60 | 82.42 → 55.26 | 109.24 unfinished → 137.00 | 141.97 → 89.71 |
| locked_chain16 | 12.55 → 12.06 | 21.36 → 15.93 | 79.80 → 50.71 | 53.31 → 30.18 |
| inconsistent | 27.80 → 23.83 | 63.58 → 32.93 | 66.56 → 63.39 | 38.14 → 33.18 |
| chain16_s0 | 13.09 → 14.59 | 23.58 → 17.66 | 91.02 → 56.54 | 57.98 → 34.44 |
| chain64_s0 | 179.89 → 152.38 | 115.93 → 83.33 | 627.01 → 349.91 | 508.78 → 286.34 |
| chain64_s1 | 141.84 → 129.62 | 114.14 → 75.51 | 664.81 → 358.21 | 409.89 → 235.87 |
| chain256_s0 | 7604.32 → 5878.99 | 678.50 → 506.06 | 7057.77 → 4846.69 | 1885.33 unfinished → 2778.19 |
| pendulum_drag | 10.69 → 11.18 | 24.75 → 18.51 | 37.39 → 30.52 | 25.08 → 23.15 |
| slider_drag | 2.15 → 1.63 | 5.95 → 4.48 | 6.34 → 6.59 | 4.16 → 3.79 |
| four_bar_drag | 13.10 → 13.71 | 26.59 → 15.08 | 43.40 → 31.90 | 24.74 → 18.48 |
| chain16_drag | 119.40 → 111.68 | 172.42 → 122.77 | 914.17 → 279.77 | 299.07 → 194.29 |

Small timing differences should not be treated as a universal win. Raw min/max and all samples are available; background machine activity was not controlled. The dense crossover microbenchmark showed that Cholesky overhead loses at 6 and 18 unknowns and pays off at 96, 192, 384, and 1536. See `dense-factorization-crossover.json`.

## Accuracy and stopping behavior

All optimized methods remain finite, and all feasible workloads meet the existing engine's convergence flag. The inconsistent assembly remains unconverged. This flag is not a proof that the requested tolerance was reached: the existing engine accepts joint residuals below 1e-6 after the outer cap. Mixed joints reach approximately 1.73e-7 in 50 iterations, not the requested 1e-9. The outer engine's stopping rules were preserved.

The original LSMR mixed-joint solve repeatedly stopped at its condition-number limit; column scaling removes those failures. Original CG on chain 256 rejected 17 capped solves and remained unfinished. Optimized CG converges in 39 outer iterations, with 68,585 inner iterations. An unfinished original solve's shorter wall time is not a speedup to the same answer.

For workloads completed on both sides, the largest final origin-entry difference is 5.176e-09 and rotation-entry difference is 2.706e-07. Maximum final rotation orthogonality error is 1.570e-15. The new pinned, deliberately scaled linear-equation tests require relative normal-equation residual below 2e-9 and compare against an independent LU reference.

All four methods have essentially identical pointer errors. Pendulum drag is about 1.15e-8; slider drag 5.0e-12; four-bar drag 1.21e-9. The chain-16 trajectory retains about 6.35e-6 maximum target error within the 20-iteration budget. That misses its requested 1e-6 pointer tolerance on both original and optimized code, although joints remain closed to about 1.4e-9. It is a remaining outer drag-policy limitation, not a performance win claimed as exact tracking.

## Verification

49 tests passed: the new linear-method tests, the complete architecture suite, dragging, refactor regressions, and JAX backend tests. Coverage includes duplicate and cancelling endpoint contributions, zero values becoming nonzero again, mixed row widths, locks, both pin layouts, cache invalidation and bounded retention, differently scaled columns, small damping, strided scratch buffers, singular fallback, nonfinite input, and failed CG iteration accounting. No automatic switching between methods or changes to the GUI default were introduced.

## Reproduce

```powershell
& 'C:/miniconda/envs/mbd_preproc/python.exe' -B tests/benchmark_linear_methods.py --paired --repeats 5 --large-repeats 3 --timeout 180 --profile --output Documentation/linear-methods-recheck.json
& 'C:/miniconda/envs/mbd_preproc/python.exe' -B -m unittest tests.test_linear_methods tests.test_architecture tests.test_dragging tests.test_refactor_regressions tests.test_jax_backend -q
```

The paired runner loads the exact original implementations saved under `Documentation/linear-methods-baseline/`; their hashes match the initial sweep. Use `--baseline-source Documentation/linear-methods-baseline` to run just the original code. Use `--cases` and `--methods` for targeted reruns.

Dense is the appropriate default for the small mechanisms tested here. SuperLU merits selection for larger sparse assemblies. CG now completes the tested large chain, but remains slower than SuperLU. LSMR is more robust after scaling and much faster on chain dragging, but did not win any of these measured workloads. Automatic selection should use representative assembly topologies and loop density, rather than a cutoff inferred from open chains alone.
