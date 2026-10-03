# Kinematics evaluator experiments — 2026-10-03

This note collects the results from the NumPy, Warp, JAX, solver-step, and
Kamino investigations. Their temporary benchmark programs and experimental
backends have been removed; the JAX CPU evaluator is the only optional
accelerated evaluator retained in the project.

## JAX CPU evaluator

The JAX evaluator compiles joint residual and Jacobian evaluation for each
prepared model. The Levenberg–Marquardt loop, dense linear solve, and work
buffers remain in the shared NumPy engine. Each evaluation transfers results
back to NumPy because that engine consumes NumPy arrays. JAX therefore speeds
up only part of the solve, and it does not change the solver's convergence
behavior.

The isolated comparison used 300 warm samples per case and backend. Each
backend ran in a fresh Python process to avoid runtime/thread-pool
interference. Runs used CPU, float64, one requested BLAS thread, Python 3.12.5,
NumPy 2.3.4, JAX 0.11.2, and Warp 1.17.0. JAX compilation is outside timed
warm samples. CUDA was unavailable.

| Case | NumPy median (ms) | JAX median (ms) | Warp median (ms) | JAX improvement vs NumPy | JAX compile (s) |
|---|---:|---:|---:|---:|---:|
| Pendulum | 0.878 | 0.831 | 1.801 | 5.4% | 0.494 |
| Slider | 0.843 | 0.745 | 1.457 | 11.7% | 0.507 |
| Four-bar | 3.009 | 2.239 | 2.952 | 25.6% | 0.583 |
| Chain 4 | 3.228 | 3.140 | 3.108 | 2.7% | 0.600 |
| Drag trajectory | 22.460 | 20.762 | 48.885 | 7.6% | 0.368 |
| Chain 16 | 26.454 | 18.907 | 16.888 | 28.5% | 0.671 |

JAX's warm median beat NumPy in all six cases, but the chain-4 gain was small.
Warp beat JAX on chain 4 and chain 16; JAX beat Warp in the other four cases.
Compilation takes hundreds of milliseconds for each model. Based on median
time savings, JAX would need roughly 10,500 repeated solves for a pendulum,
5,200 for a slider, 760 for a four-bar, 220 drag trajectories, or 90 chain-16
solves to recover compilation time compared with NumPy. These are approximate
break-even counts, not guarantees.

Assembly cases had the same iteration counts across evaluators. In the drag
trajectory, all backends used 120 total iterations and reached none of the four
targets under the compatibility step rule. This confirms that changing the
evaluator alone does not fix drag convergence. Residual/Jacobian parity was
checked for every joint type; full pendulum and four-bar solves matched the
legacy solver's iteration counts, residuals, and poses.

## Earlier NumPy and Warp comparison

The earlier two-run comparison used 200 pooled warm samples per case, split
across two fresh-process runs, and reported median / p95 milliseconds. Fixture
setup and warmup were excluded; runtime copies and state commits were included.

| Case | Legacy median / p95 | NumPy median / p95 | Warp median / p95 | Warp median change vs NumPy |
|---|---:|---:|---:|---:|
| Pendulum | 1.091 / 1.547 | 1.015 / 1.277 | 2.410 / 4.171 | 137.5% slower |
| Slider | 1.009 / 2.337 | 0.940 / 1.128 | 1.906 / 2.328 | 102.7% slower |
| Four-bar | 4.545 / 7.510 | 3.429 / 5.298 | 3.723 / 5.391 | 8.6% slower |
| Chain 4 | 5.007 / 7.881 | 3.728 / 5.382 | 3.988 / 6.537 | 7.0% slower |
| Chain 16 | 42.485 / 55.258 | 30.241 / 40.443 | 20.040 / 29.938 | 33.7% faster |
| Drag pendulum | 28.753 / 37.280 | 24.887 / 33.459 | 60.726 / 73.136 | 144.0% slower |

The tests showed NumPy is preferable on small mechanisms, while Warp helped
the chain-16 case. Warp's evaluator launch/wrapping cost outweighed compiled
arithmetic on the smaller cases and drag trajectory. At the existing 30
iterations per drag target, all backends missed the target tolerance. A
NumPy-only budget check reached all four targets when allowed 100 iterations
per target, so this was a budget/convergence issue rather than an evaluator
speed issue.

## Solver step-rule experiment

An opt-in trust-region Levenberg–Marquardt update compared actual and
linearized-predicted cost reduction to tune damping. It was tested with the
NumPy evaluator, float64, one requested BLAS thread, and 100 timed samples per
case and strategy. This code was later removed; the compatibility rule remains
the solver behavior.

| Case | Compatibility median / p95 (ms) | Trust-region median / p95 (ms) | Median iterations, compatibility → trust-region |
|---|---:|---:|---:|
| Pendulum | 1.080 / 1.977 | 1.091 / 1.548 | 5 → 5 |
| Slider | 1.003 / 2.553 | 1.012 / 1.361 | 4 → 4 |
| Four-bar | 3.735 / 5.402 | 3.194 / 5.005 | 6 → 5 |
| Chain 4 | 4.060 / 6.395 | 3.454 / 5.717 | 6 → 5 |
| Chain 16 | 36.461 / 47.518 | 30.970 / 40.291 | 11 → 9 |
| Drag, 30 iterations per target | 28.296 / 38.155 | 28.589 / 37.645 | 120 → 120 |

At 30 iterations neither rule reached a drag target. Trust-region reduced the
median positional miss from 0.478 m to 0.181 m and the worst miss from 0.667 m
to 0.230 m. In a separate 60-iteration-per-target comparison, all four
targets were reached on every trust-region trajectory, while none of the
compatibility trajectories reached all targets. Trust-region used a median
208 iterations / 48.71 ms versus 240 / 52.98 ms. It improved the larger cases
modestly by reducing iterations, but did not give a meaningful median speed
gain on the small cases. The experiment did not justify changing the default.

## Kamino assessment

NVIDIA Newton's [SolverKamino](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverKamino.html)
is a maximal-coordinate rigid-body dynamics solver. Its forward-kinematics
mode can solve looped, passive, over- or under-actuated joint systems, but the
editor's drag operation gives an arbitrary body pose target. That task-space
objective is not a direct drop-in for Kamino's documented FK inputs. Reuse
would need a Newton model adapter and an inverse-kinematics mapping, and Newton
was not installed in the tested environment. Kamino may be useful later if the
editor needs joint-coordinate inputs or FK initialization.

## Current decision

Keep the existing application solver and the optional JAX evaluator. Keep
NumPy in the project because the application solver, shared LM engine, and
work buffers depend on it; the removed NumPy component was only the separate
experimental evaluator backend. Remove Warp and the trust-region branch until
there is a new experiment with a measured benefit. JAX is opt-in because its
warm speed gains may not repay per-model compilation unless a mechanism is
solved repeatedly. The project's optional JAX install note is
`requirements-jax-experiment.txt`. The official JAX guide currently marks
Windows x86-64 CPU wheels as experimental:
[JAX installation guide](https://docs.jax.dev/en/latest/installation.html).
