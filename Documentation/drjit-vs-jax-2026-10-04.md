# Dr.Jit versus JAX

Date: 4 October 2026. This file registers the decision gates before comparative measurements. Production stays on JAX unless a later change, separate from this experiment, adopts something else.

The gates below are the operational reading of `Documentation/plan-drjit-vs-jax.md`. They are not revised after the tables are filled.

## Decision gates

Thresholds:

- Adopt a replacement only when every condition holds.
  - Correctness and lifecycle tests pass, including residuals, blocks, assembly and drag outcomes, iteration counts, accepted and rejected steps, and the GUI worker's snapshot behavior.
  - The supported application environment can load the pinned Dr.Jit build. Packaging cost is part of the decision, not a footnote.
  - On every representative workload, the warm median improves by at least 20% and by at least 1 ms against improved JAX, not only against the untouched baseline.
  - That improvement is present in every process round.
  - On every small model, p95 does not regress by more than 10% against improved JAX.
- Representative workloads are pendulum, slider, four-bar, chain of 4, the user four-bar project, and the pendulum drag trajectory. Small models are pendulum, slider, four-bar, and chain of 4.
- The decision statistic for a warm latency is the median of the per-round medians. p95 uses the median of the per-round p95 values. Samples inside one process are not treated as independent.
- Retain an optional backend when the adopt rule fails, a large model (chain 16, 64, or 256) still shows a reproducible crossover, and startup and memory stay bounded.
- Startup-focused candidate when the warm gain misses the adopt rule, but the first interactive use (cold process through the first solve) improves by at least 30% and by at least 100 ms, without a material warm or p95 regression. Packaging is evaluated before any startup-only adoption.
- Keep JAX when the gain is within round-to-round noise, a representative workload regresses, or conversion and the linear solve dominate the wall time.

Session cost, for N operations after the process exists:

`session cost(N) = import + compile + preparation + N * warm cost`

The crossover N is solved from both backends' costs. Compilation is not charged only to JAX. Mixed edit/solve sessions that invalidate kernels or bindings are included. A cheaper cold start with a slower warm path can cross over, after which the other backend wins.

Evaluator-only ceiling, computed before treating a kernel speedup as a solve speedup:

`total speedup = 1 / ((1 - f) + f / s)`

`f` is the share of solve wall time spent inside `evaluate`, measured by a wrapper, not by adding `PhaseTrace` buckets. Those buckets overlap, and drag correction calls `evaluate` outside the `evaluate` phase.

## Measurement scope fixed with the gates

Chain 64 and chain 256 are assembly scaling cases. Drag is measured on the pendulum, slider, four-bar, chain of 4, chain of 16, the disconnected assembly, and the user project. The four-way variant comparison covers the small group and chains 16 and 64. Chain 256 is compared as equivalent JAX against equivalent Dr.Jit in the headline runs, not as the improved pair. A dense factorization at chain 64 and 256 is already visible in the assembly timing and in the ceiling fraction.

Headline solver timing uses the production diagnostics policy and `trace=False`. A separate diagnostics-off run is reported beside it. Phase traces are collected on single calls and are not added together. Kernel history is a diagnostic run, not the headline clock.

Each round is a fresh process. Backend order alternates. JAX compilation caches for those processes live in a temporary directory created for the run. The user cache is not deleted. Dr.Jit has no documented cache-directory setting in the pinned build; the report records `jit_cache_dir()` when the runtime exposes it and does not flush the process-wide kernel cache.

Controlled threading is one thread for OpenMP, OpenBLAS, MKL, NumExpr, and the XLA intra-op pool, and `dr.set_thread_count(1)`. Application defaults are a later repeat, and only for cases that are close to a gate.

## What this experiment changes

JAX remains the production evaluator. `make_solver` and `get_evaluator` still accept only JAX. The Dr.Jit class is opt-in. Benchmarks attach it with `solver.evaluator = ...` after `KinematicSolver` is constructed.

## Environment

Recorded before the comparative tables. Repository `8a518a8f33eee12542809116fe745e690396e863` on `main`.

| Item | Value |
| --- | --- |
| OS | Windows 11 Home 10.0.22631, AMD64 |
| CPU | Intel Core Ultra 7 155U, 12 cores, 14 logical processors, max 2100 MHz |
| Memory | 31.45 GB |
| Power | Balanced (`381b4222-f694-41f0-9685-ff5bb260df2e`). The plan was not changed. |
| App interpreter | `C:\miniconda\envs\mbd_preproc\python.exe`, Python 3.10.19 conda-forge |
| NumPy / SciPy / JAX / jaxlib | 2.2.6 / 1.15.3 / 0.6.2 / 0.6.2 |
| Dr.Jit in the app env | Not installed |
| Experiment interpreter | `%LOCALAPPDATA%\mbd-drjit-experiment\venv`, same Python 3.10.19 via `--system-site-packages` |
| Dr.Jit | 1.5.0, `drjit.llvm` backend present when LLVM is on `PATH` |
| LLVM | Scoop clang 22.1.4. It is not bundled with the Dr.Jit wheel. |

Feasibility of the experiment: pass. The conda launcher imports Dr.Jit 1.5.0 when the venv site-packages are on `PYTHONPATH` and the scoop LLVM `bin` directory is on `PATH`. The same launcher, with every PATH entry containing `llvm` removed, imports the package and then reports the LLVM backend unavailable (`jitc_llvm_init(): LLVM API initialization failed`). The application environment itself has no `drjit` module. Adopting Dr.Jit would mean shipping the wheel and an LLVM runtime. Python was not upgraded for this comparison. The raw probe is `experiments/drjit_vs_jax/results/environment.json`.

## JAX baseline and ceiling

One controlled-thread process, diagnostics policy, 30 warm samples. The walls below are the perturbed-start assembly series (`restore` to the snapshot taken before the first timed solve). They are a single process, not the five-round decision statistic. Retrace counts on that series were zero. Source files: `experiments/drjit_vs_jax/results/ceiling-controlled-r0-jax-small.json`, `ceiling-controlled-r0-jax-chain16.json`, `ceiling-controlled-r0-jax-chain64.json`.

| Workload | Median | p95 | Iterations | Residual norm |
| --- | --- | --- | --- | --- |
| Pendulum | 1.39 ms | 1.70 ms | 5 | 1.69e-12 |
| Slider | 1.08 ms | 1.57 ms | 4 | 2.02e-10 |
| Four-bar | 2.56 ms | 4.95 ms | 5 | 1.81e-10 |
| Chain 4 | 4.67 ms | 8.63 ms | 6 | 9.67e-12 |
| User project | 4.98 ms | 7.87 ms | 5 | 8.43e-13 |
| Disconnected | 5.39 ms | 8.73 ms | 9 | 2.16e-11 |
| Redundant weld | 0.453 ms | 1.16 ms | 1 | 0 |
| Mixed joints | 33.0 ms | 49.7 ms | 50 | 6.77e-7 |
| Inconsistent weld | 15.4 ms | 18.2 ms | 50 | 0.406 |
| Chain 16 | 23.4 ms | 29.7 ms | 12 | 4.55e-13 |
| Chain 64 | 781 ms | 836 ms | 20 | 8.44e-14 |

Every row except the inconsistent weld had `converged` true. Mixed joints hit the 50-iteration cap; the flag is still true because the engine accepts a max residual under 1e-6 after the loop. The inconsistent weld stayed unconverged at the cap. Peak working set was 384 MiB for the small group, 249 MiB for chain 16, and 244 MiB for chain 64.

Chain 256 produced no ceiling samples. The ceiling command passed `--warm 30`, and the process limit in that run was 240 s. The file `ceiling-controlled-r0-jax-chain256.json` is `timeout after 240s`. The same limit killed `headline-controlled-r0-jax-chain256.json` at 360 s, before the worker could write a payload.

One later traced JAX solve of the perturbed chain, controlled threads, diagnostics policy, is the chain 256 profile. It converged in 38 iterations to residual 6.71e-14 in 662 s.

| Phase | Time | Share of the solve |
| --- | --- | --- |
| Prepare | 0.005 s | |
| Snapshot | 0.001 s | |
| Evaluate | 0.233 s | 0.035% |
| Assemble | 0.353 s | 0.05% |
| Linear | 25.3 s | 3.8% |
| Pose update | 1.41 s | 0.2% |
| Diagnostics | 635 s | 95.8% |
| Materialize | 0.162 s | |

The diagnostics phase is the redundancy SVD in `analyze_jacobian`: one factorization of the 1536 by 1536 Jacobian, then one more per joint because a revolute row block leaves the row count above the rank. That work is NumPy, shared by both evaluators. The evaluate bucket above does not include the single extra evaluation inside diagnostics. A free evaluator would save about 0.23 s and cannot produce a 20% faster solve. The repeated chain-256 rounds were not run. This traced solve is the only chain-256 measurement.

The wrapper fraction in these files is not the perturbed-start solve. `_ceiling` recaptured poses after assembly had committed, and after drag on cases that share one state. Models that had just converged were then timed as a fresh solve of an already satisfied pose: the wrapper saw 3 `evaluate` calls, and the pendulum wall fell from 1.39 ms to 0.33 ms. The chain 16 and chain 64 wrapper walls, 9.27 ms and 643 ms, match the one-iteration phase dump in the same files (rank diagnostics 7.97 ms and 706 ms). Mixed joints and the inconsistent weld stay at 50 iterations, and their wrapper walls match the assembly walls. The phase dump restored poses after the committed solve as well, so it is not a perturbed-start profile.

`_ceiling` now receives the pre-solve snapshot and records iteration counts. A corrected JAX-only ceiling rerun is still outstanding. Until that rerun, `f` and the free-evaluator bound are not reported, and the discarded wrapper fractions are not a gate input.

## Comparative results

The campaign was cancelled before the scheduled matrix finished. Gates above are unchanged. These numbers are the finished headline processes only. They are not a completed gate decision: round 4 Dr.Jit was killed before it wrote a file, and the improved-JAX variants, edit, lifecycle, cold-start, kernel, and corrected-ceiling runs were not started. Smoke results are not included.

Finished files are rounds 0–3 for both backends, plus JAX round 4, for the small group, chain 16, and chain 64. Every finished case was finite. Iteration count and the converged flag match across backends in every paired round. The inconsistent weld stays unconverged at 50 iterations on both. Fast-math was off on every Dr.Jit process. Raw files are under `experiments/drjit_vs_jax/results/headline-controlled-r*`.

The statistic is the median of the per-round medians. The range is the min and max of those round medians. Assembly times are one perturbed-start solve. Drag time is one whole trajectory.

| Workload | JAX | Dr.Jit |
| --- | --- | --- |
| Pendulum assembly | 4.32 ms (1.58–4.35), 5 rounds | 6.14 ms (5.99–6.47), 4 rounds |
| Slider assembly | 3.40 ms (1.17–3.52) | 3.22 ms (1.61–8.04) |
| Four-bar assembly | 7.01 ms (3.10–7.25) | 6.41 ms (3.01–15.1) |
| Chain 4 assembly | 8.35 ms (3.91–9.43) | 5.00 ms (3.83–25.6) |
| User project assembly | 8.31 ms (3.93–8.60) | 9.12 ms (3.43–18.0) |
| Pendulum drag trajectory | 104 ms (38.7–107) | 136 ms (53.1–309) |
| Chain 16 assembly | 53.2 ms (32.2–54.8), 5 rounds | 51.5 ms (44.0–60.8), 4 rounds |
| Chain 64 assembly | 902 ms (685–1338) | 1681 ms (1130–3089) |

Pendulum assembly is slower with Dr.Jit in every finished round, by about 1.7–4.9 ms. That already misses the adopt rule, which requires every representative workload to improve by at least 20% and at least 1 ms against improved JAX. Improved JAX was not measured. Slider, four-bar, and chain 4 are not a consistent win either: the early Dr.Jit rounds are slower than the paired JAX rounds, and the later rounds are faster. Round-to-round spread is large on both backends. JAX round 0 is the fast outlier (pendulum 1.58 ms, process 144 s). Later JAX small-group processes took about 325 s.

Chain 256 JAX headline timed out at 360 s with no samples. The separate traced solve above is the chain-256 record. Dr.Jit was not timed on that chain.
