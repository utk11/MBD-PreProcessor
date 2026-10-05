# Sparse linear-step comparison

The tables below describe the original implementations. See the
[5 October testing and optimization report](linear-methods-2026-10-05.md)
for the current code, complete-solve comparisons of all four methods,
accuracy checks, and reproducible before/after measurements.

Measured on 3 October 2026 with the conda environment `mbd_preproc`
(Python 3.10, NumPy 2.2.6, SciPy 1.15.3, JAX 0.6.2). The production
solver still uses the dense step. These prototypes are not the default.

The comparison is one damped normal-equation step at λ = 1e-3 and joint
weight 1e3, on the Jacobian JAX produced for a perturbed fixture:

`D = diag(max(diag(J.T @ J), 1e-12))`
`(J.T @ J + λ D) delta = -J.T r`

SuperLU factors that matrix. LSMR solves the augmented system
`[J; sqrt(λ) sqrt(D)]` with a zero residual block. It does not use
`lsmr(J, -r, damp=sqrt(λ))`. The default SciPy iteration cap
`min(rows, columns)` stopped these steps at `istop=7` before the
tolerance, so the prototype allows `max(8 * n, 64)` iterations and accepts
`istop` 0, 1, 2, 4, and 5.

`tests/test_architecture.py` checks the three steps against each other.
The SuperLU and LSMR deltas matched the dense delta within 1e-8 on the
pendulum, the four-bar, and the redundant fixed-joint fixture. A separate
pattern check covers a repeated endpoint (both scatter columns 0, values
added), a locked endpoint (scatter column -1 omitted), and pin rows.

Times below are the mean of five calls after the Jacobian already exists.
They are the linear step only, not compilation or the rest of the iteration.

| Fixture | Dense (s) | SuperLU (s) | LSMR (s) |
| --- | ---: | ---: | ---: |
| Pendulum | 0.000483 | 0.002115 | 0.002651 |
| Four-bar | 0.000193 | 0.002899 | 0.004332 |
| Redundant fixed joints | 0.000127 | 0.001670 | 0.001360 |

On these small systems the dense factorization is several times faster.
A repeat of the same five-call measurement stayed in the same range:
dense under 0.001 s, and the sparse steps about 0.002–0.007 s.
Sparse assembly and the iterative solve do not pay for themselves here.
The dense strategy stays the default until a larger assembly shows a
measured improvement.

## Matrix-free conjugate gradient

Measured on 4 October 2026 in the same environment. The production solver
is still the dense step. `linear_solver="cg"` applies

`v -> J.T @ (J @ v) + lambda * D * v`

with SciPy conjugate gradient. It assembles the weighted sparse Jacobian
once per step and does not allocate the normal matrix. Stopping uses
`rtol=1e-10`, `atol=1e-12`, and at most `min(4 * n, 500)` iterations.
The registered preconditioner is Jacobi, using the exact diagonal of the
damped normal matrix. A 6-by-6 block Jacobi preconditioner was timed as
well and is not the default.

One linear step, mean of five calls after the blocks exist, at
`lambda = 1e-3` and joint weight `1e3`. The step column is the largest
absolute difference from the dense delta.

| Fixture | Dense (s) | Jacobi CG (s) | CG iterations | Max step difference | Block CG (s) | Block iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Pendulum | 0.000228 | 0.000537 | 5 | 4.3e-15 | 0.000574 | 1 |
| Four-bar | 0.000100 | 0.001008 | 17 | 5.2e-15 | 0.001856 | 13 |
| Chain 16 | 0.001542 | 0.003734 | 86 | 8.6e-13 | 0.032396 | 134 |
| Chain 64 | 0.091897 | 0.014306 | 297 | 4.2e-10 | 0.215965 | 264 |

One perturbed-start assembly, diagnostics off, tolerance `1e-9`. The CG
column uses Jacobi.

| Fixture | Dense (s) | Dense iterations | Jacobi CG (s) | CG iterations | Residual, both |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pendulum | 0.002231 | 5 | 0.005905 | 5 | 1.69e-12 |
| Four-bar | 0.002251 | 5 | 0.007311 | 5 | 1.81e-10 |
| Chain 16 | 0.020697 | 12 | 0.058987 | 12 | 4.55e-13 |
| Chain 64 | 0.167572 | 20 | 0.3575 | 20 | 8.4e-14 |

Chain 256, same assembly settings: dense converged in 38 iterations and
13.71 s, residual `5.18e-14`. Jacobi CG stopped at the 50-iteration cap
in 1.98 s with residual `7.58e-4` and did not converge. Block Jacobi
failed earlier, on the chain of 64: 50 iterations, residual `2.27e-5`.

Jacobi CG matches the dense assembly on the pendulum, four-bar, and
chains of 16 and 64, and it is slower on each of those full solves. The
chain-64 linear step alone is faster, and that is not enough to move the
default. Dense stays the production linear solver.
