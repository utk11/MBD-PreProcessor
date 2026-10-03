"""Rank and whole-joint redundancy, matching the legacy SVD policy.

A dependent scalar row is not, by itself, reported as a redundant joint.
A joint is listed only when deleting its entire residual block leaves the
numerical rank unchanged. The rank threshold is

    max(tol, sigma_max * 1e-8)

with ``tol`` defaulting to ``1e-8``. Rank is configuration-dependent, so this
module does not cache a result by topology.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


DEFAULT_RANK_TOL = 1e-8


def _rank(singular_values: np.ndarray, tol: float) -> int:
    if singular_values.size == 0:
        return 0
    threshold = max(tol, float(singular_values[0]) * 1e-8)
    return int(np.sum(singular_values > threshold))


def analyze_jacobian(
    jacobian: np.ndarray,
    joint_names: Sequence[str],
    row_counts: Sequence[int],
    n_movable: int,
    tol: float = DEFAULT_RANK_TOL,
) -> Tuple[List[str], int, int]:
    """Return ``(redundant_joint_names, dof, rank)``.

    With no movable bodies the mobility is 0. With movable bodies and an empty
    residual the mobility is ``6 * n_movable``, matching ``KinematicSolver._analyze``.
    """
    if n_movable <= 0:
        return [], 0, 0
    n_unknown = 6 * int(n_movable)
    if jacobian.size == 0:
        return [], n_unknown, 0
    singular = np.linalg.svd(jacobian, compute_uv=False)
    rank = _rank(singular, tol)
    n_eq = jacobian.shape[0]
    redundant: List[str] = []
    if n_eq > rank:
        start = 0
        for name, count in zip(joint_names, row_counts):
            count = int(count)
            kept = list(range(0, start)) + list(range(start + count, n_eq))
            if kept:
                reduced = jacobian[kept, :]
                singular_reduced = np.linalg.svd(reduced, compute_uv=False)
                rank_reduced = _rank(singular_reduced, tol)
            else:
                rank_reduced = 0
            if rank_reduced == rank:
                redundant.append(str(name))
            start += count
    dof = max(0, n_unknown - rank)
    return redundant, dof, rank
