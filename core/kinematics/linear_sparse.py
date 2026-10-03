"""Sparse linear-step prototypes.

These strategies solve the same damped normal equations as the dense step.
They are not the production default. SuperLU factors the explicitly formed
damped normal matrix. LSMR solves the augmented least-squares problem with
diagonal damping rows, which is not the same as ``lsmr(..., damp=sqrt(lambda))``.

Patterns are built from endpoint blocks and scatter columns. Duplicate
contributions are accumulated by the COO constructor. Locked endpoints
(negative scatter columns) are omitted. Pin rows are included. The dense
normal workspace is not used.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from core.kinematics.linear import DIAG_FLOOR, StepResult


class _Pattern:
    def __init__(self, rows: np.ndarray, cols: np.ndarray, nrows: int, ncols: int):
        self.rows = rows
        self.cols = cols
        self.nrows = nrows
        self.ncols = ncols


def _pattern_key(plan, model, n_joint_rows: int, pin_column: Optional[int], pin_width: int):
    indices = np.asarray(plan.joint_indices, dtype=np.int32)
    widths = np.asarray(model.n_rows, dtype=np.int32)[indices] if indices.size else np.zeros(0, np.int32)
    return (
        int(plan.ncols),
        int(n_joint_rows),
        None if pin_column is None else int(pin_column),
        int(pin_width),
        np.asarray(plan.scatter_col, dtype=np.int32).tobytes(),
        indices.tobytes(),
        np.asarray(plan.row_offset, dtype=np.int32).tobytes(),
        widths.tobytes(),
    )


def _build_pattern(plan, model, n_joint_rows: int, pin_column: Optional[int], pin_width: int) -> _Pattern:
    rows = []
    cols = []
    indices = np.asarray(plan.joint_indices, dtype=np.int32).tolist()
    for local, joint_index in enumerate(indices):
        row0 = int(plan.row_offset[local])
        width = int(model.n_rows[joint_index])
        for endpoint in (0, 1):
            col0 = int(plan.scatter_col[local, endpoint])
            if col0 < 0:
                continue
            for row in range(width):
                for column in range(6):
                    rows.append(row0 + row)
                    cols.append(col0 + column)
    if pin_width and pin_column is not None:
        for row in range(int(pin_width)):
            rows.append(int(n_joint_rows) + row)
            cols.append(int(pin_column) + row)
    ncols = int(plan.ncols)
    nrows = int(n_joint_rows) + (int(pin_width) if pin_column is not None else 0)
    return _Pattern(
        np.asarray(rows, dtype=np.int32),
        np.asarray(cols, dtype=np.int32),
        nrows,
        ncols,
    )


def _fill_data(
    plan,
    model,
    blocks: np.ndarray,
    joint_weight: float,
    pin_column: Optional[int],
    pin_width: int,
    pin_sqrt: float,
) -> np.ndarray:
    data = []
    indices = np.asarray(plan.joint_indices, dtype=np.int32).tolist()
    for local, joint_index in enumerate(indices):
        width = int(model.n_rows[joint_index])
        for endpoint in (0, 1):
            col0 = int(plan.scatter_col[local, endpoint])
            if col0 < 0:
                continue
            block = blocks[joint_index, endpoint]
            for row in range(width):
                for column in range(6):
                    data.append(float(joint_weight) * float(block[row, column]))
    if pin_width and pin_column is not None:
        for row in range(int(pin_width)):
            sign = 1.0 if row < 3 else -1.0
            data.append(sign * float(pin_sqrt))
    return np.asarray(data, dtype=np.float64)


def assemble_weighted_coo(
    plan,
    model,
    blocks: np.ndarray,
    joint_weight: float,
    n_joint_rows: int,
    pin_column: Optional[int],
    pin_width: int,
    pin_sqrt: float,
    pattern: Optional[_Pattern] = None,
) -> Tuple[sp.csr_matrix, _Pattern]:
    """Build the weighted Jacobian, summing repeated endpoint contributions."""
    if pattern is None:
        pattern = _build_pattern(plan, model, n_joint_rows, pin_column, pin_width)
    data = _fill_data(plan, model, blocks, joint_weight, pin_column, pin_width, pin_sqrt)
    if pattern.rows.size == 0:
        matrix = sp.csr_matrix((pattern.nrows, pattern.ncols), dtype=np.float64)
    else:
        matrix = sp.coo_matrix(
            (data, (pattern.rows, pattern.cols)),
            shape=(pattern.nrows, pattern.ncols),
            dtype=np.float64,
        ).tocsr()
    return matrix, pattern


def _normal_diagonal(jacobian: sp.csr_matrix) -> np.ndarray:
    squared = jacobian.multiply(jacobian)
    diagonal = np.asarray(squared.sum(axis=0)).ravel()
    return np.maximum(diagonal, DIAG_FLOOR)


def _failed(reason: str, delta_out: np.ndarray, ncols: int) -> StepResult:
    if ncols and delta_out.shape[0] >= ncols:
        delta_out[:ncols] = 0.0
    return StepResult(
        step_norm=float("inf"),
        used_lstsq=False,
        finite=False,
        linear_residual=float("inf"),
        factorization="",
        failure_reason=reason,
    )


class SuperLUNormalStrategy:
    """Sparse SuperLU on the damped normal matrix. Factors are rebuilt every call."""

    name = "superlu"
    uses_blocks = True

    def __init__(self):
        self._key = None
        self._pattern: Optional[_Pattern] = None
        self.fill_nnz = 0
        self.jacobian_nnz = 0

    def solve_from_blocks(
        self,
        plan,
        model,
        blocks: np.ndarray,
        residual: np.ndarray,
        damping: float,
        joint_weight: float,
        n_joint_rows: int,
        pin_column: Optional[int],
        pin_width: int,
        pin_sqrt: float,
        delta_out: np.ndarray,
    ) -> StepResult:
        key = _pattern_key(plan, model, n_joint_rows, pin_column, pin_width)
        if key != self._key:
            self._pattern = _build_pattern(plan, model, n_joint_rows, pin_column, pin_width)
            self._key = key
        jacobian, self._pattern = assemble_weighted_coo(
            plan, model, blocks, joint_weight, n_joint_rows,
            pin_column, pin_width, pin_sqrt, self._pattern,
        )
        ncols = int(plan.ncols)
        self.jacobian_nnz = int(jacobian.nnz)
        if ncols == 0:
            return StepResult(0.0, False, True, 0.0, 0, "superlu", "")
        if residual.shape[0] != jacobian.shape[0]:
            return _failed(
                f"Residual length {residual.shape[0]} does not match Jacobian rows {jacobian.shape[0]}.",
                delta_out, ncols,
            )
        diagonal = _normal_diagonal(jacobian)
        normal = (jacobian.T @ jacobian).tocsc()
        normal = normal + sp.diags(float(damping) * diagonal, format="csc")
        self.fill_nnz = int(normal.nnz)
        rhs = -(jacobian.T @ np.asarray(residual, dtype=np.float64))
        try:
            factor = spla.splu(normal)
            solved = np.asarray(factor.solve(np.asarray(rhs, dtype=np.float64)), dtype=np.float64).reshape(ncols)
        except Exception as exc:
            return _failed(f"SuperLU failed: {exc}", delta_out, ncols)
        return _accept(solved, normal, rhs, delta_out, "superlu")


class AugmentedLSMRStrategy:
    """LSMR on ``[J; sqrt(lambda) sqrt(D)]`` with a zero padding of the residual."""

    name = "lsmr"
    uses_blocks = True

    def __init__(self):
        self._key = None
        self._pattern: Optional[_Pattern] = None
        self.fill_nnz = 0
        self.jacobian_nnz = 0

    def solve_from_blocks(
        self,
        plan,
        model,
        blocks: np.ndarray,
        residual: np.ndarray,
        damping: float,
        joint_weight: float,
        n_joint_rows: int,
        pin_column: Optional[int],
        pin_width: int,
        pin_sqrt: float,
        delta_out: np.ndarray,
    ) -> StepResult:
        key = _pattern_key(plan, model, n_joint_rows, pin_column, pin_width)
        if key != self._key:
            self._pattern = _build_pattern(plan, model, n_joint_rows, pin_column, pin_width)
            self._key = key
        jacobian, self._pattern = assemble_weighted_coo(
            plan, model, blocks, joint_weight, n_joint_rows,
            pin_column, pin_width, pin_sqrt, self._pattern,
        )
        ncols = int(plan.ncols)
        self.jacobian_nnz = int(jacobian.nnz)
        self.fill_nnz = int(jacobian.nnz) + ncols
        if ncols == 0:
            return StepResult(0.0, False, True, 0.0, 0, "lsmr", "")
        if residual.shape[0] != jacobian.shape[0]:
            return _failed(
                f"Residual length {residual.shape[0]} does not match Jacobian rows {jacobian.shape[0]}.",
                delta_out, ncols,
            )
        diagonal = _normal_diagonal(jacobian)
        damp_rows = sp.diags(np.sqrt(float(damping)) * np.sqrt(diagonal), format="csr")
        augmented = sp.vstack([jacobian, damp_rows], format="csr")
        right_hand = np.concatenate([
            -np.asarray(residual, dtype=np.float64),
            np.zeros(ncols, dtype=np.float64),
        ])
        try:
            solved, istop, iterations, norm_r, *_rest = spla.lsmr(
                augmented,
                right_hand,
                atol=1e-12,
                btol=1e-12,
                maxiter=max(8 * ncols, 64),
            )
        except Exception as exc:
            return _failed(f"LSMR failed: {exc}", delta_out, ncols)
        solved = np.asarray(solved, dtype=np.float64).reshape(ncols)
        # 0 is an exact zero solution. 1 and 2 meet atol/btol. 4 and 5 are
        # the same tests at machine precision. 7 is the iteration cap.
        if int(istop) not in (0, 1, 2, 4, 5) or not np.isfinite(solved).all():
            return _failed(f"LSMR stopped with istop={int(istop)}", delta_out, ncols)
        delta_out[:ncols] = solved
        return StepResult(
            step_norm=float(np.linalg.norm(solved)),
            used_lstsq=True,
            finite=True,
            linear_residual=float(norm_r),
            iterations=int(iterations),
            factorization="lsmr-augmented",
            failure_reason="",
        )


def _accept(solved: np.ndarray, normal, rhs: np.ndarray, delta_out: np.ndarray, name: str) -> StepResult:
    ncols = solved.shape[0]
    if not np.isfinite(solved).all():
        return _failed(f"{name} returned a non-finite step", delta_out, ncols)
    delta_out[:ncols] = solved
    residual = normal @ solved - np.asarray(rhs, dtype=np.float64)
    return StepResult(
        step_norm=float(np.linalg.norm(solved)),
        used_lstsq=False,
        finite=True,
        linear_residual=float(np.linalg.norm(residual)),
        iterations=1,
        factorization=name,
        failure_reason="",
    )


def make_linear_strategy(name: str):
    """Return a strategy. ``dense`` is the only production default."""
    key = str(name).strip().lower()
    if key == "dense":
        from core.kinematics.linear import DenseLinearStrategy
        return DenseLinearStrategy()
    if key == "superlu":
        return SuperLUNormalStrategy()
    if key == "lsmr":
        return AugmentedLSMRStrategy()
    raise ValueError("linear strategy must be 'dense', 'superlu', or 'lsmr'.")
