"""Headless tests for the kinematic solver.

Run with:  python tests/test_kinematics.py

These tests build small mechanisms from RigidBody/State/Joint directly (no
OCC shapes, no GUI) and verify that the solver satisfies joint constraints.

  * pendulum        ground + 1 body, revolute  -> body rotates about joint
  * slider          ground + 1 body, prismatic -> body translates along axis
  * four-bar        ground + 3 bodies, 4 revolute (a CLOSED LOOP)
  * over-constrained two fixed joints on one body -> redundancy reported
  * jacobian check  analytic Jacobian vs finite differences
"""

import os
import sys

import numpy as np

# Make repo root importable when run as a script.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.data_structures import Frame, Joint, JointType, RigidBody, State
from core.kinematics import KinematicSolver, capture_joint_markers
from core.kinematics import markers as M
from core.kinematics.constraints import JointConstraint


GROUND_POSE = (np.zeros(3), np.eye(3))
TOL = 1e-6


def make_body(body_id, origin, R=None):
    b = RigidBody(body_id, None, name=f"Body_{body_id}")
    b.center_of_mass = np.asarray(origin, dtype=float)
    b.local_frame = Frame(origin=np.asarray(origin, dtype=float),
                          rotation_matrix=(R if R is not None else np.eye(3)),
                          name=f"Body_{body_id}_frame")
    return b


def make_state(bodies):
    st = State()
    for b in bodies:
        R = b.local_frame.rotation_matrix if b.local_frame else np.eye(3)
        st.set_body_pose(b.id, b.local_frame.origin, R)
        b.state = st
    return st


def Rz(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def joint_world_frame(origin, R=None):
    return Frame(origin=np.asarray(origin, dtype=float),
                 rotation_matrix=(R if R is not None else np.eye(3)),
                 name="J")


def pose_of(state, body_id):
    p = state.get_body_pose(body_id)
    return p.origin, p.rotation_matrix


def test_pendulum():
    """Ground+body revolute at origin; body dragged out should rotate back onto axis."""
    # Body 1 hangs below a revolute joint at the world origin.
    body = make_body(1, origin=[1.0, 0.0, -1.0])
    state = make_state([body])

    jf = joint_world_frame([0.0, 0.0, 0.0])  # revolute axis +Z through origin
    j = Joint("hinge", JointType.REVOLUTE, -1, 1, jf, axis="+Z")
    capture_joint_markers(j, GROUND_POSE, pose_of(state, 1))

    solver = KinematicSolver([body], [j], state)
    # Perturb the body off a clean pendulum pose, then solve.
    o, R = pose_of(state, 1)
    state.set_body_pose(1, o + np.array([0.3, 0.4, 0.2]), Rz(15) @ R)

    rep = solver.solve_assembly()
    assert rep.converged, f"pendulum did not converge: {rep.message}"
    assert rep.per_joint_residual["hinge"] < 1e-6, \
        f"pendulum residual too large: {rep.per_joint_residual}"

    # Constraint check: marker origins must coincide (marker1=ground, marker2=body).
    o1, R1 = pose_of(state, 1)
    w1 = M.marker_world(j.marker1, GROUND_POSE[0], GROUND_POSE[1])
    w2 = M.marker_world(j.marker2, o1, R1)
    assert np.linalg.norm(w1.origin - w2.origin) < 1e-6
    print("PASS test_pendulum  (residual=%.2e, dof=%s)" %
          (rep.per_joint_residual["hinge"], rep.dof))


def test_slider():
    """Ground+body prismatic along +Z: off-axis drift removed, orientation restored."""
    body = make_body(1, origin=[0.0, 0.0, 0.5])
    state = make_state([body])

    jf = joint_world_frame([0.0, 0.0, 0.0])  # prismatic along Z
    j = Joint("slide", JointType.PRISMATIC, -1, 1, jf, axis="+Z")
    capture_joint_markers(j, GROUND_POSE, pose_of(state, 1))

    solver = KinematicSolver([body], [j], state)
    o, R = pose_of(state, 1)
    # Push off-axis and rotate; solver should bring it back onto the Z axis,
    # aligned, free only to slide along Z.
    state.set_body_pose(1, o + np.array([0.4, -0.3, 0.1]), Rz(30) @ R)

    rep = solver.solve_assembly()
    assert rep.converged, f"slider did not converge: {rep.message}"
    assert rep.per_joint_residual["slide"] < 1e-6

    o1, R1 = pose_of(state, 1)
    # On-axis: body marker world x,y should match ground marker (both ~0).
    w2 = M.marker_world(j.marker2, o1, R1)
    assert abs(w2.origin[0]) < 1e-6 and abs(w2.origin[1]) < 1e-6, \
        f"slider off axis: {w2.origin}"
    print("PASS test_slider    (residual=%.2e, dof=%s)" %
          (rep.per_joint_residual["slide"], rep.dof))


def test_four_bar():
    """Closed-loop four-bar: ground + crank + coupler + rocker, 4 revolute joints.

    Geometry (all in the XZ plane, axes along +Y):
      ground pivot A = (0,0,0)      -- crank (body1) length 1
      ground pivot B = (3,0,0)      -- rocker (body3) length 1
      coupler (body2) length 3 connects crank tip to rocker tip.
    A consistent closed configuration is built, then perturbed and solved.
    """
    # Consistent closed config: crank vertical, rocker vertical.
    # crank: pivot A at origin, tip at (0,0,1); put COM at tip for simplicity.
    crank = make_body(1, origin=[0.0, 0.0, 1.0])
    rocker = make_body(3, origin=[3.0, 0.0, 1.0])
    coupler = make_body(2, origin=[1.5, 0.0, 1.0])  # spans between the two tips
    state = make_state([crank, coupler, rocker])

    # Joints (axes +Y, into the plane).
    jA = Joint("A", JointType.REVOLUTE, -1, 1, joint_world_frame([0, 0, 0]), "+Y")
    jB = Joint("B", JointType.REVOLUTE, 1, 2, joint_world_frame([0, 0, 1]), "+Y")
    jC = Joint("C", JointType.REVOLUTE, 2, 3, joint_world_frame([3, 0, 1]), "+Y")
    jD = Joint("D", JointType.REVOLUTE, 3, -1, joint_world_frame([3, 0, 0]), "+Y")
    joints = [jA, jB, jC, jD]

    # Capture markers from the consistent poses.
    capture_joint_markers(jA, GROUND_POSE, pose_of(state, 1))
    capture_joint_markers(jB, pose_of(state, 1), pose_of(state, 2))
    capture_joint_markers(jC, pose_of(state, 2), pose_of(state, 3))
    capture_joint_markers(jD, pose_of(state, 3), GROUND_POSE)

    # Verify a consistent config yields ~zero residuals before perturbing.
    solver0 = KinematicSolver([crank, coupler, rocker], joints, state)
    rep0 = solver0.solve_assembly(max_iters=1)
    for name, res in rep0.per_joint_residual.items():
        assert res < 1e-6, f"four-bar initial config inconsistent at {name}: {res}"

    # Perturb all three bodies (break the loop) and solve.
    for bid, (dO, ang) in {1: ([0.2, 0.1, 0.0], 8),
                           2: ([0.0, -0.2, 0.15], -12),
                           3: ([-0.15, 0.05, 0.1], 20)}.items():
        o, R = pose_of(state, bid)
        # rotate about the mechanism axis (Y) for realism
        t = np.radians(ang)
        Ry = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
        state.set_body_pose(bid, o + np.asarray(dO, float), Ry @ R)

    solver = KinematicSolver([crank, coupler, rocker], joints, state)
    rep = solver.solve_assembly(max_iters=200, tol=1e-10)
    assert rep.converged, f"four-bar did not converge: {rep.message} {rep.per_joint_residual}"
    for name, res in rep.per_joint_residual.items():
        assert res < 1e-6, f"four-bar joint {name} residual {res}"
    # A 4-bar with ground has mobility 1.
    assert rep.dof == 1, f"four-bar dof expected 1, got {rep.dof}"
    print("PASS test_four_bar  (dof=%s, residuals=%s)" %
          (rep.dof, {k: round(v, 9) for k, v in rep.per_joint_residual.items()}))


def test_over_constrained():
    """One body fixed to ground twice -> reported as redundant, still solvable."""
    body = make_body(1, origin=[0.0, 0.0, 0.0])
    state = make_state([body])

    j1 = Joint("weld1", JointType.FIXED, -1, 1, joint_world_frame([0, 0, 0]), "+Z")
    j2 = Joint("weld2", JointType.FIXED, -1, 1, joint_world_frame([0, 0, 0]), "+Z")
    capture_joint_markers(j1, GROUND_POSE, pose_of(state, 1))
    capture_joint_markers(j2, GROUND_POSE, pose_of(state, 1))

    solver = KinematicSolver([body], [j1, j2], state)
    rep = solver.solve_assembly()
    assert rep.converged
    # Two fixed joints on one body: 12 equations, 6 unknowns -> 6 redundant.
    assert rep.dof == 0, f"expected dof 0, got {rep.dof}"
    assert len(rep.redundant_joints) >= 1, \
        f"expected a redundant joint, got {rep.redundant_joints}"
    print("PASS test_over_constrained  (redundant=%s, dof=%s)" %
          (rep.redundant_joints, rep.dof))


def test_jacobian_finite_difference():
    """Analytic per-joint Jacobians match central finite differences."""
    rng = np.random.default_rng(42)
    for jt in [JointType.REVOLUTE, JointType.PRISMATIC,
               JointType.CYLINDRICAL, JointType.SPHERICAL, JointType.FIXED]:
        # Random but moderate poses for both bodies.
        b1 = make_body(1, origin=rng.uniform(-1, 1, 3))
        b2 = make_body(2, origin=rng.uniform(-1, 1, 3))
        state = make_state([b1, b2])
        jf = joint_world_frame(rng.uniform(-0.5, 0.5, 3))
        j = Joint(f"j_{jt.name}", jt, 1, 2, jf, axis="+Z")
        capture_joint_markers(j, pose_of(state, 1), pose_of(state, 2))

        con = JointConstraint(j, lambda i: pose_of(state, i))
        J_ana, movable = con.jacobian()
        assert movable == [1, 2]

        eps = 1e-7
        J_num = np.zeros_like(J_ana)
        for k, bid in enumerate(movable):
            for d in range(6):
                delta = np.zeros(6)
                delta[d] = eps
                o0, R0 = pose_of(state, bid)
                op, Rp = M.apply_increment(o0, R0, delta)
                state.set_body_pose(bid, op, Rp)
                rp = con.residual()
                om, Rm = M.apply_increment(o0, R0, -delta)
                state.set_body_pose(bid, om, Rm)
                rm = con.residual()
                state.set_body_pose(bid, o0, R0)
                J_num[:, 6 * k + d] = (rp - rm) / (2 * eps)

        err = np.max(np.abs(J_ana - J_num))
        # Orientation rows use an O(delta) approximation; allow a modest tol.
        assert err < 1e-3, f"{jt.name}: max jac err {err:.3e}\nANA\n{J_ana}\nNUM\n{J_num}"
        print("PASS test_jacobian  %-11s (max err %.2e)" % (jt.name, err))


def test_drag_pin():
    """Drag a pendulum body; joint stays satisfied while body swings toward target."""
    body = make_body(1, origin=[1.0, 0.0, -1.0])
    state = make_state([body])
    jf = joint_world_frame([0.0, 0.0, 0.0])
    j = Joint("hinge", JointType.REVOLUTE, -1, 1, jf, axis="+Z")
    capture_joint_markers(j, GROUND_POSE, pose_of(state, 1))

    solver = KinematicSolver([body], [j], state)
    target = np.array([0.0, 1.5, -1.0])  # outside the circle — soft pin
    rep = solver.solve_drag(1, target, pin_weight=1.0, max_iters=50, tol=1e-8)
    assert rep.per_joint_residual["hinge"] < 1e-4, rep.per_joint_residual

    o1, R1 = pose_of(state, 1)
    w2 = M.marker_world(j.marker2, o1, R1)
    assert np.linalg.norm(w2.origin) < 1e-4
    assert o1[1] > 0.2, f"expected swing toward +Y, got {o1}"
    print("PASS test_drag_pin  (residual=%.2e, body=%s)" %
          (rep.per_joint_residual["hinge"], np.round(o1, 4)))


def run_all():
    test_jacobian_finite_difference()
    test_pendulum()
    test_slider()
    test_four_bar()
    test_over_constrained()
    test_drag_pin()
    print("\nAll kinematics tests passed.")


if __name__ == "__main__":
    run_all()
