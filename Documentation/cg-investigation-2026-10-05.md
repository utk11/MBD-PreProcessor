# Why conjugate gradient is slower here

This records the initial investigation before code optimization. The current
results are in the [all-method testing and optimization report](linear-methods-2026-10-05.md).

Investigated on 5 October 2026. The equations in the CG implementation match
the dense damped normal equations. The measured problems are repeated Python
and sparse-object overhead, many inner iterations with diagonal Jacobi
preconditioning, and a hard iteration cap that prevents convergence on the
large chain. CG is not automatically faster merely because the Jacobian is
sparse.

Production solver code and the existing in-progress changes were left intact.
The experimental changes are confined to `tests/benchmark_cg_investigation.py`.

## Measurement

Application interpreter: `C:/miniconda/envs/mbd_preproc/python.exe`, Python
3.10.19, NumPy 2.2.6, SciPy 1.15.3, JAX 0.6.2. This is a CPU comparison.
BLAS/OpenMP thread limits were set to one before imports. Each number is the
median of three warm, complete assembly solves in one process. Each solve
starts from the identical perturbed fixture, with `commit=False`, diagnostics
off, outer iteration limit 50, and tolerance 1e-9. Chain perturbations use
seed 0. Fixture construction, prewarming, and an initial warm solve are outside
the samples. Phase tracing and lightweight iteration instrumentation are
included. Profiles are separate untimed diagnostic runs.

The final run was performed after the regression tests completed; no other
benchmark or test was started alongside it. Background machine activity was
not controlled. Three samples characterize these fixtures, not a universal
size threshold. Four-bar dense samples ranged from 2.8 to 5.5 ms; chain-64
dense samples ranged from 184 to 216 ms and CG from 467 to 538 ms.

Raw samples, phase timings, every damping value, failed steps, iteration
counts, final poses, and profiles are in `cg-investigation-results.json`.

| Fixture | Unknowns | Dense, ms | Original CG, ms | Cache transpose only, ms | Faster assembly + cache + higher cap, ms | SuperLU, ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Pendulum | 6 | 1.7 | 4.3 | 3.2 | 3.3 | 4.5 |
| Four-bar | 18 | 4.1 | 9.9 | 7.9 | 7.0 | 6.8 |
| Chain 16 | 96 | 18.8 | 73.8 | 52.4 | 55.2 | 41.6 |
| Chain 64 | 384 | 203.0 | 498.5 | 288.0 | 348.9 | 151.7 |
| Chain 256 | 1536 | 7748.7 | 1936.0 **incomplete** | 1513.9 **incomplete** | 2644.5 | 700.8 |

All results except capped CG on chain 256 converged. Its lower elapsed time
cannot be counted as a speedup to a completed solution. Independently timed
variants vary: on chain 64 the combined optimization and transpose-only
variant perform the same inner iterations and return the same poses. The
table does not establish that vectorized assembly is slower than scalar
assembly; the separate linear phase and whole-solve timings have different
amounts of machine noise.

## Findings in the code

1. **A sparse transpose object is recreated for every operator application.**
   `CgNormalStrategy.solve_from_blocks` evaluates `jacobian.T` inside its
   `matvec` closure (`linear_sparse.py`, around line 347). On chain 64 there
   are 6,663 CG iterations over 20 outer iterations, plus applications for
   final residual checks. The separate profile records 6,723 sparse transpose
   calls, taking about 0.300 s cumulatively out of 0.875 s profiled wall time.
   These are object construction and format checks, not copies of all the
   underlying matrix values. Profiling magnifies Python costs; the timings
   above, rather than the profile, measure the speedup. Caching `transpose =
   jacobian.T` once per step alone reduced chain-64 wall time by about 42%,
   without changing inner iterations or final poses.

2. **Sparse numerical assembly still loops over every scalar in Python.**
   `_fill_data` loops over joints, endpoints, rows, and six columns and appends
   Python floats. Caching row/column patterns does not cache this work or the
   COO-to-CSR conversion. It also stores explicit zero entries in the blocks.
   The experimental fast variant collects weighted block slices with NumPy,
   removes explicit zeros after duplicate accumulation, and caches the
   transpose. With the original cap, it takes 315 ms on chain 64 and 1125 ms
   on chain 256; the latter still does not converge. Assembly improvements
   alone do not fix the cap or the number of iterations.

3. **The chosen preconditioner does not remove coupling along the chain.**
   Diagonal Jacobi rescales individual unknowns but does not solve the
   coupling between neighboring bodies. At `rtol=1e-10`, original CG performs
   25 inner iterations per pendulum assembly, 85 for the four-bar, 932 for
   chain 16, and 6,663 for chain 64. Chain-64 individual steps require
   278–375 iterations. Each iteration runs two sparse matrix-vector products
   and the SciPy/Python iteration machinery. Dense solves a small system
   through compiled numerical routines. Saving the normal matrix's storage
   is not the same as saving elapsed time.

4. **The 500-iteration cap changes outer-solver progress.**
   The limit is `min(4*ncols, 500)`. On chain 256, 17 of 50 linear solves hit
   500 iterations. The implementation discards the unfinished step via
   `_failed`; the engine increases damping by four and retries on the next
   outer iteration. Consequently CG ends at residual norm 7.75e-4, while
   dense converges in 39 outer iterations to 7.63e-14. Allowing `4*ncols`
   iterations restores convergence in 39 outer iterations, with 68,585 total
   inner iterations. Higher cap alone takes 5064 ms; higher cap plus the
   assembly/transpose changes takes 2644 ms. This is a convergence fix with
   a measurable cost, not an argument to accept an unfinished solve.

5. **A single initial linear step is not the full workload.**
   The earlier comparison used damping 1e-3 for a single step. The engine
   halves damping after accepted steps, as far as 1e-9. Chain-64 damping
   reaches 1.91e-9. Lower regularization and later Jacobians make initial-step
   timings a poor predictor of total iterative work. The earlier helper also
   assembled the dense Jacobian outside its timer while sparse assembly was
   inside the sparse timer; this investigation times complete solves on
   both sides.

SciPy defines successful CG convergence by the residual criterion and reports
positive `info` when convergence was not achieved. Effective preconditioning
can reduce iteration counts. See the [SciPy 1.15.3 CG documentation](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.sparse.linalg.cg.html).

## Recommended changes

- Keep dense for small assemblies. The four-bar is only an 18-unknown system;
  sparse setup and iterative overhead do not pay off in these samples.
- Cache the transpose immediately when optimizing CG. Replace scalar sparse
  data filling and eliminate explicit zeros after accumulating duplicates.
  Consider caching the CSR structure as a further, separately measured step.
- Replace the fixed 500 cap with a measured size-aware policy and a sparse
  direct fallback if CG cannot converge. Expose failed and successful inner
  iteration counts in solver traces. Raising the cap is insufficient to make
  CG the fastest option on these fixtures.
- Evaluate the existing SuperLU strategy for larger components. It completes
  chain 256 about 11 times faster than dense and 3.8 times faster than the
  optimized convergent CG variant here. Long chains favor sparse direct
  factorization; assemblies with more loops can have different fill and
  crossover behavior. Choose any automatic cutoff using representative
  assemblies and dragging, not this one chain family alone.
- Changing tolerance or preconditioning requires another convergence study.
  Do not simply loosen `rtol` and assume that final poses or outer iterations
  will remain equivalent. The optional body-block preconditioner performs two
  dense solves per body on every application and is not the registered path.

The GUI session constructor currently does not pass `linear_solver`, so the
app still selects dense. `linear_solver="cg"` and `"superlu"` are available
through the solver API. GUI assembly requests also enable rank diagnostics;
those costs are intentionally excluded here. These results concern the
numerical assembly solve, not rendering or full GUI latency.

## Verification and reproduction

The 27 tests in `SparseLinearTests`, `tests.test_dragging`, and
`tests.test_refactor_regressions` passed against the current implementation.
Converged experimental CG final poses differ from dense by at most 1.8e-10
in origin entries and 1.3e-10 in rotation entries across the measured cases.
SuperLU differences are below 7e-14. Tests were not run concurrently with
the final timing samples. No production solver files were edited.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
& 'C:/miniconda/envs/mbd_preproc/python.exe' -B tests/benchmark_cg_investigation.py --repeats 3 --profile --output Documentation/cg-investigation-results.json
```

The script temporarily instruments the solver within its own process. The
`cg-transpose`, `cg-fast`, `cg-cap`, and `cg-fast-cap` variants are investigation
tools, not registered production strategies.
