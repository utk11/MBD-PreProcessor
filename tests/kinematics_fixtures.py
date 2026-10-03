"""Headless mechanism builders shared by the kinematics tests and benchmarks.

Scenes are numeric: bodies, markers, and poses. No CAD geometry. Builders that
perturb a consistent closure say so in the name or the ``perturbed`` flag.
"""

from __future__ import annotations

import numpy as np

from core.data_structures import Frame, Joint, JointType, RigidBody, State
from core.kinematics import capture_joint_markers


GROUND_POSE = (np.zeros(3), np.eye(3))
TOL = 1e-6
AXES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")


def make_body(body_id, origin, R=None, name=None):
    body = RigidBody(body_id, None, name=name or f"Body_{body_id}")
    origin = np.asarray(origin, dtype=float)
    rotation = np.eye(3) if R is None else np.asarray(R, dtype=float)
    body.center_of_mass = origin.copy()
    body.local_frame = Frame(origin=origin.copy(), rotation_matrix=rotation.copy(),
                             name=f"{body.name}_frame")
    return body


def make_state(bodies):
    state = State()
    for body in bodies:
        rotation = body.local_frame.rotation_matrix if body.local_frame is not None else np.eye(3)
        state.set_body_pose(body.id, body.local_frame.origin, rotation)
        body.state = state
    return state


def Rz(deg):
    theta = np.radians(deg)
    cosine, sine = np.cos(theta), np.sin(theta)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def Ry(deg):
    theta = np.radians(deg)
    cosine, sine = np.cos(theta), np.sin(theta)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def joint_world_frame(origin, R=None):
    return Frame(origin=np.asarray(origin, dtype=float),
                 rotation_matrix=(np.eye(3) if R is None else np.asarray(R, dtype=float)),
                 name="J")


def pose_of(state, body_id):
    pose = state.get_body_pose(body_id)
    return pose.origin.copy(), pose.rotation_matrix.copy()


def copy_poses(state, body_ids):
    return {int(body_id): pose_of(state, body_id) for body_id in body_ids}


def restore_poses(state, saved):
    for body_id, (origin, rotation) in saved.items():
        state.set_body_pose(int(body_id), origin, rotation)


def _pose_or_ground(state, body_id):
    if body_id == -1:
        return GROUND_POSE
    return pose_of(state, body_id)


def build_pendulum(perturbed=True):
    body = make_body(1, origin=[1.0, 0.0, -1.0])
    state = make_state([body])
    joint = Joint("hinge", JointType.REVOLUTE, -1, 1, joint_world_frame([0.0, 0.0, 0.0]), axis="+Z")
    capture_joint_markers(joint, GROUND_POSE, pose_of(state, 1))
    if perturbed:
        origin, rotation = pose_of(state, 1)
        state.set_body_pose(1, origin + np.array([0.3, 0.4, 0.2]), Rz(15) @ rotation)
    return [body], [joint], state


def build_slider(perturbed=True):
    body = make_body(1, origin=[0.0, 0.0, 0.5])
    state = make_state([body])
    joint = Joint("slide", JointType.PRISMATIC, -1, 1, joint_world_frame([0.0, 0.0, 0.0]), axis="+Z")
    capture_joint_markers(joint, GROUND_POSE, pose_of(state, 1))
    if perturbed:
        origin, rotation = pose_of(state, 1)
        state.set_body_pose(1, origin + np.array([0.4, -0.3, 0.1]), Rz(30) @ rotation)
    return [body], [joint], state


def build_four_bar(perturbed=True):
    crank = make_body(1, origin=[0.0, 0.0, 1.0])
    coupler = make_body(2, origin=[1.5, 0.0, 1.0])
    rocker = make_body(3, origin=[3.0, 0.0, 1.0])
    bodies = [crank, coupler, rocker]
    state = make_state(bodies)
    joints = [
        Joint("A", JointType.REVOLUTE, -1, 1, joint_world_frame([0, 0, 0]), "+Y"),
        Joint("B", JointType.REVOLUTE, 1, 2, joint_world_frame([0, 0, 1]), "+Y"),
        Joint("C", JointType.REVOLUTE, 2, 3, joint_world_frame([3, 0, 1]), "+Y"),
        Joint("D", JointType.REVOLUTE, 3, -1, joint_world_frame([3, 0, 0]), "+Y"),
    ]
    capture_joint_markers(joints[0], GROUND_POSE, pose_of(state, 1))
    capture_joint_markers(joints[1], pose_of(state, 1), pose_of(state, 2))
    capture_joint_markers(joints[2], pose_of(state, 2), pose_of(state, 3))
    capture_joint_markers(joints[3], pose_of(state, 3), GROUND_POSE)
    if perturbed:
        for body_id, (shift, angle) in {1: ([0.2, 0.1, 0.0], 8),
                                        2: ([0.0, -0.2, 0.15], -12),
                                        3: ([-0.15, 0.05, 0.1], 20)}.items():
            origin, rotation = pose_of(state, body_id)
            state.set_body_pose(body_id, origin + np.asarray(shift, float), Ry(angle) @ rotation)
    return bodies, joints, state


def build_overconstrained():
    body = make_body(1, origin=[0.0, 0.0, 0.0])
    state = make_state([body])
    joints = [
        Joint("weld1", JointType.FIXED, -1, 1, joint_world_frame([0, 0, 0]), "+Z"),
        Joint("weld2", JointType.FIXED, -1, 1, joint_world_frame([0, 0, 0]), "+Z"),
    ]
    capture_joint_markers(joints[0], GROUND_POSE, pose_of(state, 1))
    capture_joint_markers(joints[1], GROUND_POSE, pose_of(state, 1))
    return [body], joints, state


def build_drag_pendulum():
    bodies, joints, state = build_pendulum(perturbed=False)
    target = np.array([0.0, 1.5, -1.0])
    return bodies, joints, state, target


def build_single_joint(joint_type, axis="+Z", ground="parent", separation=0.4):
    """One joint. ``ground`` is 'parent', 'child', or 'none'."""
    if ground == "parent":
        body_a, body_b = -1, 1
        bodies = [make_body(1, origin=[separation, 0.05, -0.1])]
    elif ground == "child":
        body_a, body_b = 1, -1
        bodies = [make_body(1, origin=[-0.2, separation, 0.15])]
    elif ground == "none":
        body_a, body_b = 1, 2
        bodies = [
            make_body(1, origin=[0.0, 0.0, 0.0]),
            make_body(2, origin=[separation, 0.1, -0.05]),
        ]
    else:
        raise ValueError(ground)
    state = make_state(bodies)
    frame = joint_world_frame([0.1, -0.05, 0.0])
    joint = Joint(f"j_{joint_type.name}_{axis}_{ground}", joint_type, body_a, body_b, frame, axis=axis)
    capture_joint_markers(joint, _pose_or_ground(state, body_a), _pose_or_ground(state, body_b))
    return bodies, [joint], state


def build_open_chain(n_bodies, joint_type=JointType.REVOLUTE, axis="+Z", spacing=1.0, perturbed=False, seed=0):
    bodies = [make_body(i, origin=[spacing * i, 0.0, 0.0]) for i in range(1, n_bodies + 1)]
    state = make_state(bodies)
    joints = []
    previous = -1
    for index, body in enumerate(bodies):
        frame = joint_world_frame([spacing * index, 0.0, 0.0])
        joint = Joint(f"j{index + 1}", joint_type, previous, body.id, frame, axis=axis)
        capture_joint_markers(joint, _pose_or_ground(state, previous), pose_of(state, body.id))
        joints.append(joint)
        previous = body.id
    if perturbed:
        rng = np.random.default_rng(seed)
        for body in bodies:
            origin, rotation = pose_of(state, body.id)
            shift = rng.normal(scale=0.02, size=3)
            state.set_body_pose(body.id, origin + shift, Rz(float(rng.normal(scale=3.0))) @ rotation)
    return bodies, joints, state


def build_two_pendulums(perturbed_first=True):
    """Two revolute pendulums that share ground and no other body."""
    first = make_body(1, origin=[1.0, 0.0, 0.0])
    second = make_body(2, origin=[-1.0, 0.0, 0.0])
    bodies = [first, second]
    state = make_state(bodies)
    left = Joint("left", JointType.REVOLUTE, -1, 1, joint_world_frame([0.0, 0.0, 0.0]), "+Z")
    right = Joint("right", JointType.REVOLUTE, -1, 2, joint_world_frame([0.0, 2.0, 0.0]), "+Z")
    capture_joint_markers(left, GROUND_POSE, pose_of(state, 1))
    capture_joint_markers(right, GROUND_POSE, pose_of(state, 2))
    if perturbed_first:
        origin, rotation = pose_of(state, 1)
        state.set_body_pose(1, origin + np.array([0.2, 0.3, 0.0]), Rz(20) @ rotation)
    return bodies, [left, right], state


def build_inconsistent_fixed():
    """Two fixed joints that demand two different body poses."""
    body = make_body(1, origin=[0.0, 0.0, 0.0])
    state = make_state([body])
    first = Joint("home", JointType.FIXED, -1, 1, joint_world_frame([0.0, 0.0, 0.0]), "+Z")
    capture_joint_markers(first, GROUND_POSE, pose_of(state, 1))
    state.set_body_pose(1, np.array([0.3, -0.2, 0.1]), Rz(25))
    second = Joint("away", JointType.FIXED, -1, 1, joint_world_frame([0.3, -0.2, 0.1], Rz(25)), "+Z")
    capture_joint_markers(second, GROUND_POSE, pose_of(state, 1))
    state.set_body_pose(1, np.array([0.05, 0.05, 0.0]), np.eye(3))
    return [body], [first, second], state


def build_drag_sequence(steps=5):
    """Reachable Z-hinge targets at z=-1, plus one unreachable jump."""
    bodies, joints, state = build_pendulum(perturbed=False)
    angles = np.linspace(10.0, 70.0, steps)
    # The initial centre is [1, 0, -1] and the hinge axis is +Z.
    # Rotation changes x/y while preserving radius one and height -1.
    targets = [np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle)), -1.0])
               for angle in angles]
    targets.append(np.array([3.0, 3.0, 0.0]))
    return bodies, joints, state, targets


def random_pose(rng, scale=0.5):
    axis = rng.normal(size=3)
    axis = axis / np.linalg.norm(axis)
    angle = float(rng.uniform(-1.5, 1.5))
    from core.kinematics import markers as M
    rotation = M.exp_so3(axis * angle)
    origin = rng.normal(scale=scale, size=3)
    return origin, rotation
