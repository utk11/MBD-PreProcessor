"""Kinematic constraint solver package.

A SolveSpace-style position-level assembly solver: full-Cartesian body-6DOF
coordinates, per-joint analytic residuals/Jacobians, and a damped
Levenberg-Marquardt Newton loop with group decomposition, dragged-body
pinning, and redundancy/DOF reporting.

Public API:
    KinematicSolver      -- the solver
    SolveReport          -- solve outcome
    capture_joint_markers-- helper to populate Joint.marker1/marker2
"""

from core.kinematics.solver import KinematicSolver, SolveReport
from core.kinematics import markers

__all__ = ["KinematicSolver", "SolveReport", "markers", "capture_joint_markers"]


def capture_joint_markers(joint, body1_pose, body2_pose):
    """Populate ``joint.marker1`` and ``joint.marker2`` from current world poses.

    body1_pose / body2_pose are ``(origin, rotation_matrix)`` tuples of the two
    connected bodies *at joint-creation time*.  For a ground body pass
    ``(np.zeros(3), np.eye(3))``.
    """
    joint.marker1 = markers.capture_marker(joint.frame, body1_pose[0], body1_pose[1])
    joint.marker2 = markers.capture_marker(joint.frame, body2_pose[0], body2_pose[1])
    return joint
