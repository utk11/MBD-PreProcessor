"""Reusable float64 buffers for one kinematic solve session.

A workspace belongs to one prepared shape and one caller. Concurrent solves
must not share it. Pose buffers keep ground in slot 0. Trial poses live beside
the accepted poses so a rejected step never needs to rewrite them.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.kinematics.prepared import PreparedModel


def _zeros(shape) -> np.ndarray:
    return np.zeros(shape, dtype=np.float64)


@dataclass
class Workspace:
    """Owned scratch for poses, residuals, blocks, and optional dense linear buffers."""

    model_revision: int
    n_slots: int
    n_joints: int
    max_rows: int
    max_cols: int
    origin: np.ndarray
    rotation: np.ndarray
    trial_origin: np.ndarray
    trial_rotation: np.ndarray
    residual: np.ndarray
    weighted: np.ndarray
    blocks: np.ndarray
    jacobian: np.ndarray
    normal: np.ndarray
    gradient: np.ndarray
    delta: np.ndarray
    system: np.ndarray
    copy_count: int = 0
    dense_linear: bool = True

    @property
    def session_id(self) -> int:
        return id(self)


def _capacity(model: PreparedModel) -> tuple:
    widest = max((plan.ncols for plan in model.components), default=0)
    widest = max(widest, model.analysis_plan.ncols)
    max_cols = int(widest)
    max_rows = int(model.total_rows) + 6
    return max_rows, max_cols


def create_workspace(model: PreparedModel, *, dense_linear: bool = True) -> Workspace:
    """Allocate reusable buffers; sparse strategies omit dense linear matrices."""
    max_rows, max_cols = _capacity(model)
    rotation = _zeros((model.n_slots, 3, 3))
    rotation[:] = np.eye(3)
    trial_rotation = np.array(rotation, copy=True)
    return Workspace(
        model_revision=model.revision,
        n_slots=model.n_slots,
        n_joints=model.n_joints,
        max_rows=max_rows,
        max_cols=max_cols,
        origin=_zeros((model.n_slots, 3)),
        rotation=rotation,
        trial_origin=_zeros((model.n_slots, 3)),
        trial_rotation=trial_rotation,
        residual=_zeros((max_rows,)),
        weighted=_zeros((max_rows,)),
        blocks=_zeros((model.n_joints, 2, 6, 6)),
        jacobian=_zeros((max_rows, max_cols) if dense_linear else (0, 0)),
        normal=_zeros((max_cols, max_cols) if dense_linear else (0, 0)),
        gradient=_zeros((max_cols,) if dense_linear else (0,)),
        delta=_zeros((max_cols,)),
        system=_zeros((max_cols, max_cols) if dense_linear else (0, 0)),
        copy_count=0,
        dense_linear=dense_linear,
    )


def workspace_matches(workspace: Workspace, model: PreparedModel, *, dense_linear: bool = True) -> bool:
    max_rows, max_cols = _capacity(model)
    return (
        workspace.n_slots == model.n_slots
        and workspace.n_joints == model.n_joints
        and workspace.max_rows == max_rows
        and workspace.max_cols == max_cols
        and workspace.blocks.shape[:1] == (model.n_joints,)
        and workspace.dense_linear == dense_linear
    )


def ensure_workspace(workspace: Workspace | None, model: PreparedModel, *, dense_linear: bool = True) -> Workspace:
    if workspace is None or not workspace_matches(workspace, model, dense_linear=dense_linear):
        return create_workspace(model, dense_linear=dense_linear)
    workspace.model_revision = model.revision
    return workspace


def assert_buffer_contract(array: np.ndarray, shape: tuple, name: str) -> None:
    """Reject a buffer that is the wrong shape, dtype, or layout."""
    if array.shape != shape:
        raise ValueError(f"{name} shape {array.shape} != {shape}")
    if array.dtype != np.float64:
        raise ValueError(f"{name} dtype {array.dtype} != float64")
    if not array.flags.c_contiguous:
        raise ValueError(f"{name} is not C-contiguous")


def validate_workspace(workspace: Workspace) -> None:
    assert_buffer_contract(workspace.origin, (workspace.n_slots, 3), "origin")
    assert_buffer_contract(workspace.rotation, (workspace.n_slots, 3, 3), "rotation")
    assert_buffer_contract(workspace.trial_origin, (workspace.n_slots, 3), "trial_origin")
    assert_buffer_contract(workspace.trial_rotation, (workspace.n_slots, 3, 3), "trial_rotation")
    assert_buffer_contract(workspace.residual, (workspace.max_rows,), "residual")
    assert_buffer_contract(workspace.weighted, (workspace.max_rows,), "weighted")
    assert_buffer_contract(
        workspace.blocks, (workspace.n_joints, 2, 6, 6), "blocks"
    )
    dense = workspace.dense_linear
    assert_buffer_contract(workspace.jacobian, (workspace.max_rows, workspace.max_cols) if dense else (0, 0), "jacobian")
    assert_buffer_contract(workspace.normal, (workspace.max_cols, workspace.max_cols) if dense else (0, 0), "normal")
    assert_buffer_contract(workspace.gradient, (workspace.max_cols,) if dense else (0,), "gradient")
    assert_buffer_contract(workspace.delta, (workspace.max_cols,), "delta")
    assert_buffer_contract(workspace.system, (workspace.max_cols, workspace.max_cols) if dense else (0, 0), "system")
