# Plan: evaluate Dr.Jit against JAX

Date: 4 October 2026. Status: investigation plan; no Dr.Jit benchmark has been run and no production backend has changed.

## Objective and initial assessment

Determine whether Dr.Jit improves interactive dragging, assembly solving, or first-use latency enough to justify maintaining or replacing the current JAX evaluator. Measure installation and maintenance costs as well as speed.

There is a plausible opportunity in batched small-vector and matrix operations, avoiding unnecessary joint calculations, and reducing first-use compilation cost. There is no evidence yet that Dr.Jit will be faster here. Dr.Jit's documented focus is large data-parallel programs; our small mechanisms may instead be dominated by dispatch, tracing, conversion, and the remaining NumPy work. [Dr.Jit overview](https://drjit.readthedocs.io/en/stable/what.html).

Start with a CPU evaluator experiment. A complete solver rewrite, GPU migration, and automatic differentiation are separate proposals that require evidence from this experiment.

## What the current code actually does

- `core/kinematics/backends/jax_cpu_backend.py` uses CPU float64, `jax.jit`, and `jax.vmap` to compute residuals and **analytic** Jacobian blocks. It does not use `jax.grad`, `jacfwd`, or `jacrev`.
- Residual-only and residual-plus-blocks paths have separate compiled functions. Each evaluation materializes NumPy results and uses a Python loop to pack the requested joints.
- Kernels process every joint in the prepared model even when `joint_indices` requests a subset. Joint-type masks select among point, line, and orientation calculations. Profiling and generated-code inspection should establish which calculations survive compilation.
- Cache keys use slot count, joint count, float64, CPU device, and formulation. Marker/topology arrays are runtime inputs, with constant binding refreshed by content signatures. The executable cache cap is eight.
- `core/kinematics/engine.py` retains iteration control, pose updates, drag correction, assembly, and linear steps in the shared host engine. Dense is the production linear strategy; sparse prototypes exist.
- JAX is now required: `make_solver` only accepts JAX, and `KinematicSolver` constructs its JAX evaluator directly. The older review's description of JAX as optional is historical.
- `requirements.txt` targets JAX 0.6.2 in the Python 3.10 application environment. The historical evaluator comparison used Python 3.12/JAX 0.11.2 and earlier solver behavior. Its numbers cannot establish the current baseline.

Reuse `tests/kinematics_fixtures.py`, `tests/test_jax_backend.py`, `tests/test_architecture.py`, and `PhaseTrace`. Treat `Documentation/jax-warp-backend-review-2026-10-03.md` and `Documentation/sparse-linear-comparison.md` as context, not measurements for this decision.

## Phase 1: establish feasibility and the performance ceiling

1. Record the repository revision, actual Python/NumPy/JAX/jaxlib/SciPy versions, Windows version, CPU and instruction support, memory, power mode, and thread settings. Confirm the application environment rather than trusting dependency comments.
2. In a disposable experiment environment matching the application's Python 3.10 and numerical packages, verify an available Dr.Jit wheel, explicit `drjit.llvm` backend availability, float64 arithmetic, and a NumPy round trip. Pin the working Dr.Jit/LLVM versions and retain the install recipe. Do not upgrade the application Python solely to run this comparison.
3. Verify LLVM runtime discovery on Windows and startup from the normal application launcher. The documented CPU backend requires an installed LLVM runtime; include that deployment cost in the decision. [Backend requirements](https://drjit.readthedocs.io/en/stable/what.html#backends).
4. Capture current JAX cold startup and warm solve times. In separate diagnostic runs, attribute time to preparation, evaluation, host conversion/packing, block assembly, linear solve, pose updates, drag correction, diagnostics, and state commit.
5. Use `PhaseTrace` for broad phases and temporary finer instrumentation for tracing/conversion/packing. The current `materialize_s` spans dispatch and host materialization, not just copying. Some evaluation occurs inside drag correction and diagnostics, so do not infer the evaluator share from the `evaluate` phase alone. Nested timings must not be added as if disjoint.

For each workload, calculate the ceiling for an evaluator-only replacement:

`total speedup = 1 / ((1 - f) + f / s)`

Here `f` is the fraction of current wall time replaced and `s` is its measured speedup. If `f = 0.20`, making that work twice as fast improves total latency by only 10%; making it free caps the speedup at 1.25x. Use this to decide how much investigation is worthwhile.

Deliverable: feasibility note, reproducible JAX baseline, and time breakdown. If the supported application environment cannot run Dr.Jit, report that blocker before porting the evaluator.

## Phase 2: build the smallest comparable prototype

Implement an experimental `DrJitCpuEvaluator` against the existing `ConstraintEvaluator` protocol, keeping the same NumPy input/output contract and float64 precision. Keep JAX as the production default and dependency.

- Use explicit LLVM types, not automatic device selection. Represent joint lanes with suitable vector/matrix types or a structure-of-arrays layout, with gathers for body endpoints.
- Port the existing residual and analytic block formulas exactly, including the `compat-v1` orientation linearization, SO(3) branch thresholds, axis signs, row widths, and output ordering. Retain separate residual-only and full paths.
- Initially compute the same all-joint workload as JAX. Preserve requested-subset packing, global block indexing, repeated-endpoint accumulation, and untouched block buffers when `write_blocks=False`.
- Pass changing poses and marker/topology data as explicit inputs. Prevent numerical values from being accidentally baked into kernels; check cache reuse on changing values. Dr.Jit's opaque inputs can avoid value-dependent recompilation. [Opaque inputs](https://drjit.readthedocs.io/en/stable/reference.html#drjit.opaque).
- Measure ordinary cached tracing and a frozen-function variant if supported by the pinned release. Cached kernels alone do not remove Dr.Jit's per-call tracing; freezing can address that overhead. Keep host conversion outside the frozen computation and pass mutable constants explicitly to avoid stale captures. Record limitations and retracing. [Function freezing](https://drjit.readthedocs.io/en/stable/freeze.html).
- Keep compile/tracing/materialization accounting and session release behavior observable. Dropping session references does not prove either runtime returns memory to the OS.

Use a benchmark-only injection helper or direct engine calls to compare evaluators. The current public factory and solver constructor are JAX-only, so changing the backend registry alone is insufficient. Avoid broad public API changes for the experiment.

Deliverable: opt-in CPU prototype and a harness that runs both evaluators through the same solver configuration.

## Phase 3: pass correctness before comparing speed

Compare Dr.Jit with both JAX and the retained `JointConstraint` formula reference. Start with existing parity thresholds: residuals `atol=1e-10, rtol=1e-9`; blocks `atol=1e-9, rtol=1e-8`.

Cover all five joint types; all signed axes; ground at either endpoint; two movable endpoints; locked and repeated endpoints; empty models; reordered/subset joint indices; spherical three-row packing; nonzero marker offsets; randomized poses; identity, small-angle, and near-pi rotations. Check finite outputs, SO(3) validity, and residual-only buffer sentinels.

Then compare complete assembly and drag outcomes: convergence/failure classification, residual norm, target error, poses, iterations, accepted/rejected steps, and linear residual. Use identical initial poses, seeds, limits, weights, damping policy, grouping, diagnostics policy, and linear strategy. Flag iteration changes even when final poses match; do not present different numerical work as a kernel speedup.

Exercise constant refresh, same-shape topology replacement, shape changes, more than eight shapes, release/recreate, and repeated sessions. Test the GUI worker's snapshot/revision behavior before any adoption.

Start with Dr.Jit fast math disabled and document the setting. Any fast-math variant is a separate candidate and must pass the same correctness gates. [Fast-math semantics](https://drjit.readthedocs.io/en/stable/reference.html#drjit.JitFlag.FastMath).

Automatic differentiation would change the existing approximate analytic linearization and is outside this parity experiment.

## Phase 4: benchmark representative work

| Workload | Purpose |
| --- | --- |
| Pendulum, slider, four-bar, chain 4 | Typical small-model overhead and closed-loop behavior |
| Perturbed chains 16, 64, 256 | Scaling and crossover; apply equal time/memory caps |
| Mixed-joint assemblies | Cost of joint-type masks and orientation branches |
| Disconnected assemblies with a small dragged component | All-joint versus requested-component work |
| Reachable drag sequence plus unreachable jump | Repeated warm use, target error, failure behavior |
| Redundant and inconsistent fixed constraints | Numerical robustness and reporting |
| At least one representative user assembly | Check that synthetic results transfer to practical use |
| Marker/axis edits, topology edits, shape churn | Cache invalidation and interactive stalls |

Use existing builders where possible and retain fixed seeds. Reinitialize assembly samples to the same perturbed pose; otherwise repeated solves mostly measure an already-converged model. For drag, restore the starting pose before each whole trajectory and preserve state between targets within it. Report reachable and unreachable targets separately.

Measure three levels:

1. Evaluator: residual-only and residual-plus-blocks, with and without conversion/packing included. Include full and subset requests.
2. Headless solver: complete assembly and drag operations, including the ordinary state commit and production diagnostics policy; report any diagnostics-off diagnostic run separately.
3. GUI: request-to-visible-pose latency and first drag after opening/editing, including queue/coalescing behavior. Advance to this check only if the headless result is promising.

Run each backend in a fresh process, alternating order across at least five rounds. Use at least 300 warm samples per small case/round; record justified smaller counts for expensive cases. Report median, p95, absolute milliseconds saved, variability across rounds, and a confidence interval for the difference. Bootstrap by process/trajectory rather than treating correlated targets as independent observations. Report both per-target and whole-trajectory drag timing.

Control and record BLAS, JAX, and Dr.Jit thread settings. First compare controlled threading, then repeat promising cases with the application's normal defaults. Disable diagnostic instrumentation for headline timing; collect phase and kernel evidence separately.

Exclude fixture construction and explicit prewarm from warm samples, but include input conversion, tracing/replay, evaluation, NumPy materialization, and packing in the main evaluator timing. Synchronize JAX outputs with `block_until_ready()` for compute-only timing. [JAX benchmarking guidance](https://docs.jax.dev/en/latest/benchmarking.html). Force Dr.Jit evaluation and completion for compute-only wall timing; use kernel history for diagnostic execution timings, keeping those distinct from total latency. [Dr.Jit timing API](https://drjit.readthedocs.io/en/stable/reference.html#drjit.kernel_history).

Keep separate measurements for cold process/import, first unseen kernel, existing disk-cache first call, and warm repeated calls. A fresh process alone is not a cold compiler cache. Isolate benchmark caches in temporary locations using the selected runtime's supported configuration; do not delete user caches. Record peak process memory and memory across repeated model lifecycles.

## Phase 5: distinguish a library gain from an algorithm/layout gain

After the equivalent port, test these independently, retaining the unchanged baseline:

1. Requested-joint-only evaluation instead of computing the whole model.
2. Joint-type grouping to avoid unused orientation/line calculations.
3. Cheaper NumPy output packing and conversion/layout handling.

Apply corresponding optimizations to JAX where feasible, including preparation and cache costs in both implementations. Compare baseline JAX, improved JAX, equivalent Dr.Jit, and improved Dr.Jit. An improvement available in both libraries does not by itself justify migration.

Consider CUDA only after a large or batched workload demonstrates enough arithmetic to amortize transfers. Report a Dr.Jit GPU comparison separately from CPU-versus-CPU results. Retaining the host solver still requires transfers every iteration. Consider compiling more of the solver only as a new scoped investigation; the coupled dense/sparse solve is not automatically handled by Dr.Jit's per-lane parallelism.

## Decision gates and final report

These are proposed engineering gates, not performance predictions. Register them before examining comparative results:

- **Adopt a replacement:** all correctness/lifecycle gates pass; deployment works in the supported environment; representative warm median latency improves at least 20% and 1 ms per solve/drag target; improvement persists across process rounds; small-model p95 does not regress more than 10%. Compare with improved JAX, not only the initial baseline.
- **Retain an optional backend:** meaningful gains occur only on large models, with a reproducible crossover and bounded startup/memory costs.
- **Startup-focused candidate:** warm improvement is small but first interactive use improves at least 30% and 100 ms without material warm/p95 regressions. Evaluate packaging costs before adopting for startup alone.
- **Keep JAX:** gains are within noise, practical workloads regress, or conversion/linear solving dominates. Recommend measured JAX or host-engine improvements instead.

For each workload report total cost over a realistic session:

`session cost(N) = startup/import/compile/preparation cost + N * warm operation cost`

Calculate the crossover from both backends' costs; do not charge compilation only to JAX. Include mixed edit/solve sessions that repeatedly invalidate kernels or bindings. A cheaper cold start and a slower warm path can also have a crossover, after which JAX wins.

Retain a runnable harness, environment recipe, raw per-sample CSV/JSON, correctness results, kernel/cache evidence, and a report table containing median/p95, cold latency, compilation count, memory, iterations, target error, and session crossover. State untested workloads explicitly.

Suggested effort: half a day for feasibility/baseline, one to two days for the equivalent port and parity, and one to two days for measurements and targeted optimizations. Stop early after a failed feasibility gate or a measured ceiling too small to justify the work. Production migration is a separate decision after the report.
