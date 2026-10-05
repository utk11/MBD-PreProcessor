"""Dense damped normal-equation step shared by every evaluator.

The compatibility step is the legacy Levenberg-Marquardt system

    (J^T J + lambda * diag(D)) delta = -J^T r

with ``D = max(diag(J^T J), 1e-12)``. That is the normal equation of

    min || [J; sqrt(lambda) * diag(sqrt(D))] delta - [-r; 0] ||_2

and it is the only linear step used in the first experiment. Both the NumPy
and Warp evaluators call this function, so they cannot silently diverge here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


DIAG_FLOOR = 1e-12


@dataclass
class StepResult:
    """One linear step. ``delta`` is written into the caller buffer."""

    step_norm: float
    used_lstsq: bool
    finite: bool
    linear_residual: float = 0.0
    iterations: Optional[int] = None
    factorization: str = "dense-lu"
    failure_reason: str = ""


def solve_dense_damped(
    jacobian: np.ndarray,
    residual: np.ndarray,
    damping: float,
    normal: np.ndarray,
    gradient: np.ndarray,
    delta: np.ndarray,
    system: np.ndarray,
) -> StepResult:
    """Solve one dense step into the provided scratch buffers.

    ``jacobian`` is ``(nrows, ncols)`` and ``residual`` is ``(nrows,)``.
    The scratch arrays must be at least ``ncols`` on each linear-system axis.
    """
    nrows, ncols = jacobian.shape
    if ncols == 0:
        return StepResult(step_norm=0.0, used_lstsq=False, finite=True)
    if residual.shape[0] != nrows:
        raise ValueError(
            f"Residual length {residual.shape[0]} does not match Jacobian rows {nrows}."
        )
    # Leading slices of the session buffers are not contiguous when this
    # component is narrower than the allocated capacity. Pack them so the
    # factorization sees the same dense system the legacy solver builds.
    packed_j = np.ascontiguousarray(jacobian, dtype=np.float64)
    packed_r = np.ascontiguousarray(residual, dtype=np.float64)
    nrm = packed_j.T @ packed_j
    grd = packed_j.T @ packed_r
    if normal.shape[0] >= ncols:
        normal[:ncols, :ncols] = nrm
    if gradient.shape[0] >= ncols:
        gradient[:ncols] = grd
    diag = np.diag(nrm).copy()
    np.maximum(diag, DIAG_FLOOR, out=diag)
    sys = nrm.copy()
    diag_index = np.arange(ncols)
    sys[diag_index, diag_index] += damping * diag
    if system.shape[0] >= ncols:
        system[:ncols, :ncols] = sys
    used_lstsq = False
    try:
        solved = np.linalg.solve(sys, -grd)
    except np.linalg.LinAlgError:
        solved = np.linalg.lstsq(sys, -grd, rcond=None)[0]
        used_lstsq = True
    out = delta[:ncols]
    out[:] = solved
    finite = bool(np.isfinite(out).all())
    step_norm = float(np.linalg.norm(out)) if finite else float("inf")
    if finite:
        linear_residual = float(np.linalg.norm(sys @ out + grd))
    else:
        linear_residual = float("inf")
    return StepResult(
        step_norm=step_norm,
        used_lstsq=used_lstsq,
        finite=finite,
        linear_residual=linear_residual,
        iterations=1,
        factorization="dense-lstsq" if used_lstsq else "dense-lu",
        failure_reason="" if finite else "Dense step was not finite",
    )


class DenseLinearStrategy:
    """Production linear step. Keeps the compatibility damped normal equations."""

    name = "dense"
    uses_blocks = False

    def solve(
        self,
        jacobian: np.ndarray,
        residual: np.ndarray,
        damping: float,
        normal: np.ndarray,
        gradient: np.ndarray,
        delta: np.ndarray,
        system: np.ndarray,
    ) -> StepResult:
        return solve_dense_damped(
            jacobian, residual, damping, normal, gradient, delta, system
        )
