"""Constraint evaluator interface. The production evaluator is JAX on CPU."""

from __future__ import annotations

from typing import Protocol

import numpy as np

from core.kinematics.prepared import ComponentPlan, PreparedModel


class ConstraintEvaluator(Protocol):
    name: str
    device: str

    def evaluate(
        self,
        model: PreparedModel,
        origin: np.ndarray,
        rotation: np.ndarray,
        joint_indices: np.ndarray,
        residual_out: np.ndarray,
        blocks_out: np.ndarray,
        write_blocks: bool,
    ) -> int:
        """Write compact residuals for ``joint_indices`` and, if requested, blocks.

        Returns the number of residual rows written. Blocks are addressed by the
        global joint index. Residual rows are packed in ``joint_indices`` order.
        """

    def prewarm(self, model: PreparedModel, origin: np.ndarray, rotation: np.ndarray) -> None:
        """Compile or touch evaluator kernels outside a timed sample."""


def get_evaluator(backend: str) -> ConstraintEvaluator:
    key = backend.strip().lower()
    if key in ("jax", "jax_cpu"):
        from core.kinematics.backends.jax_cpu_backend import JaxCpuEvaluator

        return JaxCpuEvaluator()
    raise ValueError(f"Unknown kinematics backend {backend!r}. Only 'jax' is available here.")


def assemble_blocks(
    plan: ComponentPlan,
    model: PreparedModel,
    blocks: np.ndarray,
    jacobian_out: np.ndarray,
) -> np.ndarray:
    """Scatter local ``(joint, endpoint, 6, 6)`` blocks into a dense Jacobian.

    Rows of one joint are zeroed by the caller-owned slice fill. Endpoint
    contributions accumulate, so a joint whose two ends are the same movable
    body matches the legacy ``+=`` placement.
    """
    nrows = plan.n_residual_rows
    ncols = plan.ncols
    jacobian = jacobian_out[:nrows, :ncols]
    if nrows == 0 or ncols == 0:
        if jacobian.size:
            jacobian.fill(0.0)
        return jacobian
    jacobian.fill(0.0)
    for local, j_index in enumerate(plan.joint_indices.tolist()):
        row0 = int(plan.row_offset[local])
        width = int(model.n_rows[j_index])
        for endpoint in (0, 1):
            col0 = int(plan.scatter_col[local, endpoint])
            if col0 < 0:
                continue
            jacobian[row0:row0 + width, col0:col0 + 6] += blocks[j_index, endpoint, :width, :]
    return jacobian
