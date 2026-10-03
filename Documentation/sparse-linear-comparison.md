# Sparse linear-step comparison

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