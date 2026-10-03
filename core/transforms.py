"""Shared local, reference, and world frame transforms.

``State`` owns the live body pose. ``RigidBody.local_frame`` is the reference
geometry frame (the imported centre of mass, in meters), not a second copy of
the live pose. Joint markers and attached user frames stay in their recorded
local or reference coordinates. World placement is computed here.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from core.data_structures import Frame
from core.kinematics.markers import compose


def world_from_local(
    body_origin: np.ndarray,
    body_rotation: np.ndarray,
    local_origin: np.ndarray,
    local_rotation: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Map a body-local frame through the body's current world pose."""
    origin, rotation = compose(
        np.asarray(body_origin, dtype=float),
        np.asarray(body_rotation, dtype=float),
        np.asarray(local_origin, dtype=float),
        np.asarray(local_rotation, dtype=float),
    )
    return np.asarray(origin, dtype=float), np.asarray(rotation, dtype=float)


def body_world_pose(body) -> Tuple[np.ndarray, np.ndarray]:
    """Current world origin and rotation for ``body``.

    The live ``State`` pose wins. The reference ``local_frame`` is only a
    fallback for a body that has not been placed yet.
    """
    state = getattr(body, "state", None)
    if state is not None:
        pose = state.get_body_pose(body.id)
        if pose is not None:
            return (
                np.array(pose.origin, dtype=float, copy=True),
                np.array(pose.rotation_matrix, dtype=float, copy=True),
            )
    return reference_pose(body)


def body_world_frame(body, name: str = None) -> Frame:
    """A frame for the body's current world pose. This does not write ``local_frame``."""
    origin, rotation = body_world_pose(body)
    if name is None:
        local = getattr(body, "local_frame", None)
        name = local.name if local is not None else f"Body_{body.id}_World"
    return Frame(origin=origin, rotation_matrix=rotation, name=name)


def reference_pose(body) -> Tuple[np.ndarray, np.ndarray]:
    """Imported geometry frame, in meters. Independent of the live pose."""
    local = getattr(body, "local_frame", None)
    if local is None:
        return np.zeros(3, dtype=float), np.eye(3)
    return (
        np.array(local.origin, dtype=float, copy=True),
        np.array(local.rotation_matrix, dtype=float, copy=True),
    )


def attachment_reference_frame(frame: Frame, body, coordinates: str) -> Frame:
    """Frame in imported geometry coordinates, for the renderer's delta transform."""
    if coordinates in ("world", "reference_geometry"):
        return Frame(frame.origin, frame.rotation_matrix, frame.name)
    if coordinates != "body_local" or body is None:
        raise ValueError(f"Cannot resolve attached frame '{frame.name}' in {coordinates!r}.")
    origin, rotation = world_from_local(*reference_pose(body), frame.origin, frame.rotation_matrix)
    return Frame(origin, rotation, frame.name)


def attachment_world_frame(frame: Frame, body, coordinates: str) -> Frame:
    """Owned world frame for dialogs, respecting the stored coordinate convention."""
    if coordinates == "world":
        return Frame(frame.origin, frame.rotation_matrix, frame.name)
    if body is None:
        raise ValueError(f"Frame '{frame.name}' has no parent body.")
    origin, rotation = body_world_pose(body)
    if coordinates == "body_local":
        result = world_from_local(origin, rotation, frame.origin, frame.rotation_matrix)
    elif coordinates == "reference_geometry":
        ref_origin, ref_rotation = reference_pose(body)
        delta_rotation = rotation @ ref_rotation.T
        delta_origin = origin - delta_rotation @ ref_origin
        result = world_from_local(delta_origin, delta_rotation, frame.origin, frame.rotation_matrix)
    else:
        raise ValueError(f"Unknown frame coordinate convention {coordinates!r}.")
    return Frame(*result, name=frame.name)
