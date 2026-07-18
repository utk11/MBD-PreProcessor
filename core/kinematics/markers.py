"""Marker math and SE(3) helpers for the kinematic solver.

A *marker* is a coordinate frame rigidly attached to a body, expressed in that
body's local (COM-referenced) coordinates.  Every joint owns two markers:

    marker1 : pose of the joint frame in body1's local coordinates
    marker2 : pose of the joint frame in body2's local coordinates

They are captured once, at joint-creation time, from the bodies' poses at that
instant (``M_i = T_i^{-1} . J_world``).  From then on the joint moves with the
bodies: at solve time the joint's two world frames are ``P_i = T_i . M_i`` and
the constraint residuals measure how ``P1`` relates to ``P2``.

Poses are represented as plain ``(origin, rotation_matrix)`` numpy pairs and as
4x4 homogeneous matrices where convenient.  Orientation *updates* inside the
solver use a local rotation-vector (Lie algebra) parametrization: the unknown
per body is ``delta = [dp (3), dw (3)]`` and the update is::

    p <- p + dp
    R <- exp( skew(w) ) @ R          (left / world-frame increment)

Only stdlib + numpy are used here so this module stays headless-testable.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from core.data_structures import Frame, Pose


# ---------------------------------------------------------------------------
# Axis resolution
# ---------------------------------------------------------------------------

def axis_vector(axis_str: str) -> np.ndarray:
    """Map an axis token ("+X", "-Z", ...) to a unit vector in the marker frame."""
    table = {
        "+X": np.array([1.0, 0.0, 0.0]),
        "-X": np.array([-1.0, 0.0, 0.0]),
        "+Y": np.array([0.0, 1.0, 0.0]),
        "-Y": np.array([0.0, -1.0, 0.0]),
        "+Z": np.array([0.0, 0.0, 1.0]),
        "-Z": np.array([0.0, 0.0, -1.0]),
    }
    try:
        return table[axis_str].copy()
    except KeyError:
        raise ValueError(f"Unknown axis token: {axis_str!r}")


# ---------------------------------------------------------------------------
# SO(3) helpers
# ---------------------------------------------------------------------------

def skew(w: np.ndarray) -> np.ndarray:
    """Skew-symmetric (cross-product) matrix of a 3-vector."""
    wx, wy, wz = float(w[0]), float(w[1]), float(w[2])
    return np.array([
        [0.0, -wz, wy],
        [wz, 0.0, -wx],
        [-wy, wx, 0.0],
    ])


def exp_so3(w: np.ndarray) -> np.ndarray:
    """Exponential map so(3) -> SO(3) (Rodrigues' formula)."""
    theta = float(np.linalg.norm(w))
    W = skew(w)
    if theta < 1e-12:
        # Second-order Taylor for numerical stability near zero.
        return np.eye(3) + W + 0.5 * (W @ W)
    a = np.sin(theta) / theta
    b = (1.0 - np.cos(theta)) / (theta * theta)
    return np.eye(3) + a * W + b * (W @ W)


def log_so3(R: np.ndarray) -> np.ndarray:
    """Logarithm map SO(3) -> so(3) rotation vector (principal branch)."""
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-9:
        # Near identity: w ~ vee((R - R^T)/2)
        return 0.5 * np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ])
    if abs(np.pi - theta) < 1e-6:
        # Near 180 degrees: recover axis from the symmetric part.
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0.0, None))
        # Resolve signs using off-diagonal terms.
        if A[0, 1] < 0:
            axis[1] = -axis[1]
        if A[0, 2] < 0:
            axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        if n < 1e-12:
            # Degenerate; pick any orthogonal axis.
            return np.array([np.pi, 0.0, 0.0])
        return theta * axis / n
    factor = theta / (2.0 * np.sin(theta))
    return factor * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])


def project_to_so3(R: np.ndarray) -> np.ndarray:
    """Project a near-rotation matrix onto SO(3) via SVD (removes drift)."""
    U, _, Vt = np.linalg.svd(R)
    Rp = U @ Vt
    if np.linalg.det(Rp) < 0:
        U[:, -1] *= -1.0
        Rp = U @ Vt
    return Rp


# ---------------------------------------------------------------------------
# SE(3) pose helpers  (pose == (origin, rotation_matrix))
# ---------------------------------------------------------------------------

def pose_to_matrix(origin: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from origin + rotation matrix."""
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = origin
    return T


def matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Split a 4x4 homogeneous transform into (origin, rotation_matrix)."""
    return T[0:3, 3].copy(), T[0:3, 0:3].copy()


def compose(o1: np.ndarray, R1: np.ndarray, o2: np.ndarray, R2: np.ndarray):
    """Compose two poses: (o1,R1) . (o2,R2)."""
    return o1 + R1 @ o2, R1 @ R2


def invert(o: np.ndarray, R: np.ndarray):
    """Invert a rigid pose."""
    Rt = R.T
    return -Rt @ o, Rt


def frame_to_pose(frame: Frame) -> Pose:
    """Convert a Frame to a Pose (copies arrays)."""
    return Pose(origin=frame.origin.copy(), rotation_matrix=frame.rotation_matrix.copy())


def pose_to_frame(pose: Pose, name: str = "Frame") -> Frame:
    """Convert a Pose to a Frame (copies arrays)."""
    return Frame(origin=pose.origin.copy(), rotation_matrix=pose.rotation_matrix.copy(), name=name)


def capture_marker(joint_world_frame: Frame, body_origin: np.ndarray,
                   body_R: np.ndarray) -> Frame:
    """Express a world-coordinate joint frame in a body's local coordinates.

    M = T_body^{-1} . J_world
    """
    inv_o, inv_R = invert(np.asarray(body_origin, dtype=float),
                          np.asarray(body_R, dtype=float))
    m_o, m_R = compose(inv_o, inv_R,
                       np.asarray(joint_world_frame.origin, dtype=float),
                       np.asarray(joint_world_frame.rotation_matrix, dtype=float))
    return Frame(origin=m_o, rotation_matrix=m_R,
                 name=f"{joint_world_frame.name}_local")


def marker_world(marker: Frame, body_origin: np.ndarray, body_R: np.ndarray) -> Frame:
    """Transform a body-local marker into world coordinates:  P = T_body . M."""
    w_o, w_R = compose(np.asarray(body_origin, dtype=float),
                       np.asarray(body_R, dtype=float),
                       np.asarray(marker.origin, dtype=float),
                       np.asarray(marker.rotation_matrix, dtype=float))
    return Frame(origin=w_o, rotation_matrix=w_R, name=marker.name)


# ---------------------------------------------------------------------------
# Incremental pose updates (solver unknowns)
# ---------------------------------------------------------------------------

def apply_increment(origin: np.ndarray, R: np.ndarray,
                    delta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Apply a 6-DOF increment [dp(3), dw(3)] to a pose.

    Translation is added in world frame; rotation is left-multiplied
    (world-frame increment):  R <- exp(skew(dw)) @ R.
    """
    dp = delta[0:3]
    dw = delta[3:6]
    new_origin = np.asarray(origin, dtype=float) + dp
    new_R = exp_so3(dw) @ np.asarray(R, dtype=float)
    return new_origin, new_R


def relative_rotation_vector(R1: np.ndarray, R2: np.ndarray) -> np.ndarray:
    """Rotation vector taking R1 to R2 (left increment):  R2 = exp(skew(w)) R1."""
    return log_so3(R2 @ R1.T)


# ---------------------------------------------------------------------------
# Orthonormal basis helper (for constraint projection)
# ---------------------------------------------------------------------------

def orthonormal_perp_basis(v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return two unit vectors (u1, u2) orthonormal to each other and to v."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Cannot build perpendicular basis of a near-zero vector")
    v = v / n
    # Pick the world axis least aligned with v for numerical stability.
    candidates = [np.array([1.0, 0.0, 0.0]),
                  np.array([0.0, 1.0, 0.0]),
                  np.array([0.0, 0.0, 1.0])]
    ref = min(candidates, key=lambda c: abs(float(np.dot(v, c))))
    u1 = np.cross(v, ref)
    u1 = u1 / np.linalg.norm(u1)
    u2 = np.cross(v, u1)
    u2 = u2 / np.linalg.norm(u2)
    return u1, u2
