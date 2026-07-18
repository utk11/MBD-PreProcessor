"""Per-joint constraint residuals and analytic Jacobians.

Full-Cartesian (absolute / body-6DOF) formulation.  The unknown per movable
body is a 6-vector ``delta = [dp(3), dw(3)]`` applied as::

    p <- p + dp
    R <- exp(skew(dw)) @ R          (left / world-frame increment)

Every joint contributes a stack of scalar residual equations that must vanish
when the two bodies are posed consistently with the joint.  For each residual
block we also return its analytic Jacobian with respect to the delta of the
two connected bodies (``None`` for a body that is ground / locked).

Constraint counts (position level, cross-product form):
    FIXED       6   coincident origins + coincident orientations
    REVOLUTE    6   coincident origins (3) + a1×a2 (3, rank 2 → 5 DOF removed)
    PRISMATIC   6   a1×(o2-o1) (3, rank 2) + relative orientation locked (3)
    CYLINDRICAL 6   a1×(o2-o1) (3, rank 2) + a1×a2 (3, rank 2) → 4 DOF removed
    SPHERICAL   3   coincident origins (3)

Least-squares absorbs the rank-1 deficiency in each cross-product block.
Jacobian SVD rank reports the true mobility.

This module is pure numpy and has no GUI / OCC imports so it is headless-testable.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from core.data_structures import Frame, Joint, JointType
from core.kinematics import markers as M


# Number of scalar residual rows each joint type contributes.
# Axis-alignment and point-on-line use the 3-vector cross product form
# (one dependent equation each); the LM least-squares solve absorbs the
# rank deficiency and Jacobian rank reports the true DOF.
CONSTRAINT_COUNT: Dict[JointType, int] = {
    JointType.FIXED: 6,        # origin (3) + orientation (3)
    JointType.REVOLUTE: 6,     # origin (3) + a1×a2 (3, rank 2)
    JointType.PRISMATIC: 6,    # a1×(o2-o1) (3, rank 2) + orientation (3)
    JointType.CYLINDRICAL: 6,  # a1×(o2-o1) (3, rank 2) + a1×a2 (3, rank 2)
    JointType.SPHERICAL: 3,    # origin (3)
}


class JointConstraint:
    """Residual + Jacobian provider for one joint.

    Parameters
    ----------
    joint:
        The Joint (must have ``marker1`` and ``marker2`` Frames set).
    get_pose:
        Callable ``body_id -> (origin, R)`` returning the *current* world pose.
        Not used for ground bodies (id == -1), which read ``ground_pose``.
    ground_id:
        Body id treated as ground (default -1).
    ground_pose:
        World pose of the ground body ``(origin, R)``.
    """

    def __init__(self, joint: Joint, get_pose,
                 ground_id: int = -1,
                 ground_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None):
        self.joint = joint
        self.get_pose = get_pose
        self.ground_id = ground_id
        self.ground_pose = ground_pose if ground_pose is not None else (
            np.zeros(3), np.eye(3))

        if joint.marker1 is None or joint.marker2 is None:
            raise ValueError(
                f"Joint '{joint.name}' has no local markers; capture them at "
                f"creation time before solving.")

        # Motion axis in each marker's local frame (constant).
        self._a_local = M.axis_vector(joint.axis)

    # ------------------------------------------------------------------
    # Pose access helpers
    # ------------------------------------------------------------------
    def _body_pose(self, body_id: int) -> Tuple[np.ndarray, np.ndarray]:
        if body_id == self.ground_id:
            return self.ground_pose
        return self.get_pose(body_id)

    def is_ground(self, body_id: int) -> bool:
        return body_id == self.ground_id

    # ------------------------------------------------------------------
    # World joint frames for both bodies
    # ------------------------------------------------------------------
    def _world_frames(self):
        """Return (o1, R1, o2, R2) world frames of marker1 and marker2."""
        b1 = self.joint.body1_id
        b2 = self.joint.body2_id
        bo1, bR1 = self._body_pose(b1)
        bo2, bR2 = self._body_pose(b2)
        w1 = M.marker_world(self.joint.marker1, bo1, bR1)
        w2 = M.marker_world(self.joint.marker2, bo2, bR2)
        return w1.origin, w1.rotation_matrix, w2.origin, w2.rotation_matrix

    def _world_axes(self, R1: np.ndarray, R2: np.ndarray):
        """Motion axis of the joint in world, from each body's marker."""
        a1 = R1 @ self._a_local
        a2 = R2 @ self._a_local
        return a1, a2

    # ------------------------------------------------------------------
    # Jacobian building blocks
    # ------------------------------------------------------------------
    def _point_jac(self, body_id: int, world_point: np.ndarray) -> Optional[np.ndarray]:
        """d(world_point)/d(delta_body) = [I | -skew(world_point - body_origin)].

        Returns None for ground (that body contributes no unknowns).
        """
        if body_id == self.ground_id:
            return None
        bo, _ = self._body_pose(body_id)
        J = np.zeros((3, 6))
        J[:, 0:3] = np.eye(3)
        J[:, 3:6] = -M.skew(world_point - bo)
        return J

    def _dir_jac(self, body_id: int, world_dir: np.ndarray) -> Optional[np.ndarray]:
        """d(world_dir)/d(delta_body) = [0 | -skew(world_dir)].

        Returns None for ground.
        """
        if body_id == self.ground_id:
            return None
        J = np.zeros((3, 6))
        J[:, 3:6] = -M.skew(world_dir)
        return J

    def _frame_jac(self, body_id: int) -> Optional[np.ndarray]:
        """d(orientation residual)/d(delta_body) = [0 | I].  None for ground."""
        if body_id == self.ground_id:
            return None
        J = np.zeros((3, 6))
        J[:, 3:6] = np.eye(3)
        return J

    def _accumulate(self, Jrows: np.ndarray, block: Optional[np.ndarray],
                    cols: Optional[Tuple[int, int]], which: int = 0):
        """Place a body Jacobian block into the columns described by ``cols``.

        ``cols`` is ``(start, end)`` for that body's 6 DOF columns in Jrows.
        ``which`` is unused (kept for call-site clarity) — columns come from ``cols``.
        """
        if block is None or cols is None:
            return
        c0, _c1 = cols
        Jrows[:, c0:c0 + 6] += block

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    @property
    def n_residuals(self) -> int:
        return CONSTRAINT_COUNT[self.joint.joint_type]

    def residual(self) -> np.ndarray:
        """Stacked residual vector for this joint at the current poses."""
        o1, R1, o2, R2 = self._world_frames()
        jt = self.joint.joint_type

        if jt == JointType.FIXED:
            r_pos = o1 - o2
            r_rot = M.relative_rotation_vector(R1, R2)
            return np.concatenate([r_pos, r_rot])

        if jt == JointType.SPHERICAL:
            return o1 - o2

        a1, a2 = self._world_axes(R1, R2)

        if jt == JointType.REVOLUTE:
            # Origins coincide; axes parallel (a1 × a2 = 0).
            r_pos = o1 - o2
            r_ax = np.cross(a1, a2)
            return np.concatenate([r_pos, r_ax])

        if jt == JointType.CYLINDRICAL:
            # o2 lies on axis through o1 along a1; axes parallel.
            r_line = np.cross(a1, o2 - o1)
            r_ax = np.cross(a1, a2)
            return np.concatenate([r_line, r_ax])

        if jt == JointType.PRISMATIC:
            # o2 on axis; full relative orientation locked.
            r_line = np.cross(a1, o2 - o1)
            r_rot = M.relative_rotation_vector(R1, R2)
            return np.concatenate([r_line, r_rot])

        raise ValueError(f"Unsupported joint type: {jt}")

    def jacobian(self) -> Tuple[np.ndarray, List[int]]:
        """Stacked Jacobian for this joint.

        Returns
        -------
        J : (n_residuals, 6*M) ndarray
            M = number of movable (non-ground) bodies connected (1 or 2).
        movable_body_ids : list of int
            The body ids whose 6-DOF columns appear in J, in order.
        """
        o1, R1, o2, R2 = self._world_frames()
        jt = self.joint.joint_type
        b1 = self.joint.body1_id
        b2 = self.joint.body2_id

        movable: List[int] = []
        if not self.is_ground(b1):
            movable.append(b1)
        if not self.is_ground(b2):
            movable.append(b2)

        n = self.n_residuals
        J = np.zeros((n, 6 * len(movable)))

        def cols_for(body_id: int) -> Optional[Tuple[int, int]]:
            if body_id not in movable:
                return None
            i = movable.index(body_id)
            return (6 * i, 6 * (i + 1))

        c1 = cols_for(b1)
        c2 = cols_for(b2)

        # --- origin-coincidence rows (o1 - o2) ---
        if jt in (JointType.FIXED, JointType.REVOLUTE, JointType.SPHERICAL):
            if c1 is not None:
                self._accumulate(J[0:3], self._point_jac(b1, o1), c1, 0)
            if c2 is not None:
                blk = self._point_jac(b2, o2)
                if blk is not None:
                    self._accumulate(J[0:3], -blk, c2, 1)

        if jt == JointType.FIXED:
            # r_rot = log(R2 R1^T): d/dw1 = -I, d/dw2 = +I (left increments)
            if c1 is not None:
                blk = self._frame_jac(b1)
                if blk is not None:
                    self._accumulate(J[3:6], -blk, c1, 0)
            if c2 is not None:
                blk = self._frame_jac(b2)
                if blk is not None:
                    self._accumulate(J[3:6], blk, c2, 1)
            return J, movable

        if jt == JointType.SPHERICAL:
            return J, movable

        a1, a2 = self._world_axes(R1, R2)
        Ja1 = self._dir_jac(b1, a1)
        Ja2 = self._dir_jac(b2, a2)

        # --- revolute / cylindrical axis-parallel: r = a1 × a2 ---
        # dr/da1 = -skew(a2),  dr/da2 = skew(a1)
        if jt in (JointType.REVOLUTE, JointType.CYLINDRICAL):
            row0 = 3 if jt == JointType.REVOLUTE else 3  # both use rows 3:6 for axis
            # REVOLUTE: rows 0:3 pos, 3:6 axis
            # CYLINDRICAL: rows 0:3 line, 3:6 axis
            if jt == JointType.REVOLUTE:
                row0 = 3
            else:
                row0 = 3
            if c1 is not None and Ja1 is not None:
                blk = (-M.skew(a2)) @ Ja1
                self._accumulate(J[row0:row0 + 3], blk, c1, 0)
            if c2 is not None and Ja2 is not None:
                blk = M.skew(a1) @ Ja2
                self._accumulate(J[row0:row0 + 3], blk, c2, 1)

        # --- cylindrical / prismatic point-on-line: r = a1 × (o2 - o1) ---
        # Let d = o2 - o1.  r = a1 × d = skew(a1) d
        # dr = skew(da1) d + skew(a1) dd
        #    = -skew(d) da1 + skew(a1) (do2 - do1)
        if jt in (JointType.CYLINDRICAL, JointType.PRISMATIC):
            d = o2 - o1
            Jp1 = self._point_jac(b1, o1)
            Jp2 = self._point_jac(b2, o2)
            if c1 is not None:
                if Jp1 is not None:
                    blk = M.skew(a1) @ (-Jp1)
                    self._accumulate(J[0:3], blk, c1, 0)
                if Ja1 is not None:
                    corr = (-M.skew(d)) @ Ja1
                    self._accumulate(J[0:3], corr, c1, 0)
            if c2 is not None and Jp2 is not None:
                blk = M.skew(a1) @ Jp2
                self._accumulate(J[0:3], blk, c2, 1)

        # --- prismatic relative-orientation rows ---
        if jt == JointType.PRISMATIC:
            # same sign convention as FIXED orientation residual
            if c1 is not None:
                blk = self._frame_jac(b1)
                if blk is not None:
                    self._accumulate(J[3:6], -blk, c1, 0)
            if c2 is not None:
                blk = self._frame_jac(b2)
                if blk is not None:
                    self._accumulate(J[3:6], blk, c2, 1)

        return J, movable
