"""Create two-ended joints from frames owned by their respective bodies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from core.data_structures import Frame, Joint, JointType
from core.kinematics.markers import capture_marker
from core.transforms import attachment_world_frame


AXES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
AXIAL_JOINTS = (JointType.REVOLUTE, JointType.PRISMATIC, JointType.CYLINDRICAL)


@dataclass(frozen=True)
class JointFrameOption:
    """A frame offered for one specific body, in its recorded coordinates."""

    name: str
    owner_body_id: int
    frame: Frame
    coordinates: str = "world"

    @property
    def key(self) -> Tuple[int, str, str]:
        return int(self.owner_body_id), self.name, self.coordinates

    @property
    def label(self) -> str:
        return f"{self.name} [{self.coordinates.replace('_', ' ')}]"


def axis_alignment(axis: str) -> np.ndarray:
    """Return a right-handed rotation that carries ``axis`` onto local +Z."""
    vectors = {
        "+X": np.array([1.0, 0.0, 0.0]), "-X": np.array([-1.0, 0.0, 0.0]),
        "+Y": np.array([0.0, 1.0, 0.0]), "-Y": np.array([0.0, -1.0, 0.0]),
        "+Z": np.array([0.0, 0.0, 1.0]), "-Z": np.array([0.0, 0.0, -1.0]),
    }
    if axis not in vectors:
        raise ValueError(f"Unsupported joint-frame axis {axis!r}.")
    source = vectors[axis]
    target = np.array([0.0, 0.0, 1.0])
    cross = np.cross(source, target)
    cosine = float(np.dot(source, target))
    if np.linalg.norm(cross) < 1e-12:
        if cosine > 0.0:
            return np.eye(3)
        return np.diag([1.0, -1.0, -1.0])
    skew = np.array([
        [0.0, -cross[2], cross[1]],
        [cross[2], 0.0, -cross[0]],
        [-cross[1], cross[0], 0.0],
    ])
    return np.eye(3) + skew + (skew @ skew) / (1.0 + cosine)


def _validate_frame(frame: Frame, label: str) -> None:
    origin = np.asarray(frame.origin, dtype=float)
    rotation = np.asarray(frame.rotation_matrix, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError(f"{label} has an invalid origin.")
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError(f"{label} has an invalid rotation matrix.")
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError(f"{label} must have a right-handed orthonormal orientation.")


def resolve_frame(option: JointFrameOption, owner_body, *, ground_pose=None) -> Frame:
    """Resolve one selected frame to a copied world frame at creation time."""
    if option.owner_body_id == -1:
        if ground_pose is None:
            if owner_body is None or owner_body.local_frame is None:
                origin, rotation = np.zeros(3), np.eye(3)
            else:
                source = owner_body.local_frame
                origin, rotation = source.origin, source.rotation_matrix
        else:
            origin, rotation = ground_pose
        owner = type("_GroundFrameOwner", (), {"id": -1, "local_frame": Frame(origin, rotation)})()
    else:
        owner = owner_body
    if owner is None or int(owner.id) != int(option.owner_body_id):
        raise ValueError(f"Frame '{option.name}' is not attached to its selected body.")
    _validate_frame(option.frame, f"Frame '{option.name}'")
    world = attachment_world_frame(option.frame, owner, option.coordinates)
    _validate_frame(world, f"Frame '{option.name}'")
    return Frame(np.array(world.origin, copy=True),
                 np.array(world.rotation_matrix, copy=True), option.name)


def make_joint(
    *, name: str, joint_type: JointType, body1_id: int, body2_id: int,
    frame1: JointFrameOption, frame2: JointFrameOption,
    body1=None, body2=None, pose1=None, pose2=None,
    axis1: str = "+Z", axis2: str = "+Z",
    flip1: bool = False, flip2: bool = False,
) -> Joint:
    """Build a fully marked joint; no document mutation occurs on failure."""
    name = str(name).strip()
    if not name:
        raise ValueError("Joint name cannot be empty.")
    if int(body1_id) == int(body2_id):
        raise ValueError("A joint must connect two different bodies.")
    if int(frame1.owner_body_id) != int(body1_id):
        raise ValueError(f"Frame '{frame1.name}' does not belong to Body 1.")
    if int(frame2.owner_body_id) != int(body2_id):
        raise ValueError(f"Frame '{frame2.name}' does not belong to Body 2.")

    world1 = resolve_frame(frame1, body1, ground_pose=pose1)
    world2 = resolve_frame(frame2, body2, ground_pose=pose2)
    if joint_type in AXIAL_JOINTS:
        world1.rotation_matrix = world1.rotation_matrix @ axis_alignment(axis1)
        world2.rotation_matrix = world2.rotation_matrix @ axis_alignment(axis2)
    flip_rotation = np.diag([1.0, -1.0, -1.0])
    if flip1:
        world1.rotation_matrix = world1.rotation_matrix @ flip_rotation
    if flip2:
        world2.rotation_matrix = world2.rotation_matrix @ flip_rotation

    joint = Joint(name, joint_type, int(body1_id), int(body2_id), world1, "+Z")
    local_pose1 = pose1 if pose1 is not None else _owner_pose(body1)
    local_pose2 = pose2 if pose2 is not None else _owner_pose(body2)
    joint.marker1 = capture_marker(world1, local_pose1[0], local_pose1[1])
    joint.marker2 = capture_marker(world2, local_pose2[0], local_pose2[1])
    joint.marker1.name = f"{name}_Body1"
    joint.marker2.name = f"{name}_Body2"
    joint.marker1_source = frame1.name
    joint.marker2_source = frame2.name
    joint.marker1_axis = axis1 if joint_type in AXIAL_JOINTS else None
    joint.marker2_axis = axis2 if joint_type in AXIAL_JOINTS else None
    joint.marker1_flip = bool(flip1)
    joint.marker2_flip = bool(flip2)
    return joint


def _owner_pose(body):
    if body is None:
        return np.zeros(3), np.eye(3)
    from core.transforms import body_world_pose
    return body_world_pose(body)
