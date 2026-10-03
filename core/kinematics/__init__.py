"""Kinematic constraint solver package.

A SolveSpace-style position-level assembly solver: full-Cartesian body-6DOF
coordinates, per-joint analytic residuals and Jacobians, and a damped
Levenberg-Marquardt loop. Constraint evaluation is compiled with JAX. The
iteration and the dense linear step stay in NumPy.

Public API:
    KinematicSolver       -- the only production solver session
    SolveReport           -- solve outcome, including an explicit trace
    capture_joint_markers -- helper to populate Joint.marker1/marker2
"""

from core.kinematics.reports import SolveReport
from core.kinematics.solver import KinematicSolver
from core.kinematics import markers

__all__ = [
    "KinematicSolver",
    "SolveReport",
    "make_solver",
    "markers",
    "capture_joint_markers",
]


def make_solver(bodies, joints, state, backend="jax", ground_id=-1,
                ground_pose=None, locked_body_ids=None, **options):
    """Build the JAX-backed solver session.

    ``backend`` must be ``jax``. The previous NumPy-only solver has been
    removed, and a missing JAX install raises ``ImportError`` instead of
    selecting another implementation.
    """
    key = str(backend).strip().lower()
    if key in ("legacy", "a", "existing", "numpy"):
        raise RuntimeError(
            "The legacy NumPy kinematic solver has been removed. "
            "Use the JAX-backed solver (backend='jax')."
        )
    if key not in ("jax", "jax_cpu"):
        raise ValueError("backend must be 'jax'.")
    return KinematicSolver(
        bodies, joints, state,
        ground_id=ground_id,
        ground_pose=ground_pose,
        locked_body_ids=locked_body_ids,
        **options,
    )


def capture_joint_markers(joint, body1_pose, body2_pose):
    """Populate ``joint.marker1`` and ``joint.marker2`` from current world poses.

    body1_pose / body2_pose are ``(origin, rotation_matrix)`` tuples of the two
    connected bodies at joint-creation time. For a ground body pass
    ``(np.zeros(3), np.eye(3))``.
    """
    joint.marker1 = markers.capture_marker(joint.frame, body1_pose[0], body1_pose[1])
    joint.marker2 = markers.capture_marker(joint.frame, body2_pose[0], body2_pose[1])
    return joint
