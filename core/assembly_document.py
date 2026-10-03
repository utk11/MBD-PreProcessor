"""Assembly document: one owner for bodies, joints, frames, loads, and revisions.

Widgets and the solver read this document. They do not keep a second copy of
the connectivity. Numeric work copies arrays out of it; it does not share the
live pose buffers with the worker.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

from core.data_structures import Force, Frame, Joint, RigidBody, State, Torque


@dataclass
class ChangeSet:
    """Identifiers removed by one deletion, for a single visual update."""

    deleted_body_ids: List[int] = field(default_factory=list)
    deleted_joints: List[str] = field(default_factory=list)
    deleted_frames: List[str] = field(default_factory=list)
    deleted_forces: List[str] = field(default_factory=list)
    deleted_torques: List[str] = field(default_factory=list)


class AssemblyDocument:
    """Authoritative assembly record for one open project."""

    def __init__(self):
        self.generation = 0
        self.topology_revision = 0
        self.marker_revision = 0
        self.external_pose_epoch = 0
        self.pose_revision = 0
        self.bodies: List[RigidBody] = []
        self.joints: Dict[str, Joint] = {}
        self.frames: Dict[str, Frame] = {}
        self.frame_to_body: Dict[str, int] = {}
        self.frame_coordinates: Dict[str, str] = {}
        self.forces: Dict[str, Force] = {}
        self.torques: Dict[str, Torque] = {}
        self.state: Optional[State] = None
        self.unit_scale = 1.0
        self.source_cad_path: Optional[str] = None
        self.source_cad_fingerprint: Optional[str] = None
        self.joints_by_body: Dict[int, List[str]] = {}
        self.frames_by_body: Dict[int, List[str]] = {}
        self.forces_by_body: Dict[int, List[str]] = {}
        self.torques_by_body: Dict[int, List[str]] = {}

    def begin_generation(self) -> int:
        """A new project. Results stamped with the previous generation are stale."""
        self.generation += 1
        self.topology_revision += 1
        self.marker_revision += 1
        self.external_pose_epoch += 1
        self.pose_revision += 1
        return self.generation

    def note_topology(self) -> None:
        self.topology_revision += 1

    def note_markers(self) -> None:
        self.marker_revision += 1

    def note_external_pose(self) -> None:
        """A pose edit that did not come from an accepted solver result."""
        self.external_pose_epoch += 1

    def note_pose_commit(self) -> None:
        self.pose_revision += 1

    def clear_contents(self) -> None:
        """Empty the record without advancing the generation by itself."""
        self.bodies.clear()
        self.joints.clear()
        self.frames.clear()
        self.frame_to_body.clear()
        self.frame_coordinates.clear()
        self.forces.clear()
        self.torques.clear()
        self.state = None
        self.reindex()

    def replace_bodies(self, bodies: Iterable[RigidBody]) -> None:
        self.bodies[:] = list(bodies)
        self.reindex()

    def install(self, staged: "AssemblyDocument") -> None:
        """Publish a fully restored document while preserving GUI aliases."""
        self.bodies[:] = staged.bodies
        for name in ("joints", "frames", "frame_to_body", "frame_coordinates", "forces", "torques"):
            target = getattr(self, name)
            target.clear()
            target.update(getattr(staged, name))
        self.state = staged.state
        self.unit_scale = staged.unit_scale
        self.source_cad_path = staged.source_cad_path
        self.source_cad_fingerprint = staged.source_cad_fingerprint
        self.begin_generation()
        self.reindex()

    def body_by_id(self, body_id: int) -> Optional[RigidBody]:
        for body in self.bodies:
            if int(body.id) == int(body_id):
                return body
        return None

    def constrained_body_ids(self) -> set:
        connected = set()
        for joint in self.joints.values():
            connected.add(int(joint.body1_id))
            connected.add(int(joint.body2_id))
        return connected

    def add_joint(self, joint: Joint) -> None:
        self.joints[joint.name] = joint
        self.note_topology()
        self.note_markers()
        self.reindex()

    def delete_joint(self, name: str) -> bool:
        if name not in self.joints:
            return False
        del self.joints[name]
        self.note_topology()
        self.reindex()
        return True

    def add_frame(self, frame: Frame, body_id: Optional[int] = None,
                  coordinates: str = "world") -> None:
        self.frames[frame.name] = frame
        if body_id is None:
            self.frame_to_body.pop(frame.name, None)
            self.frame_coordinates[frame.name] = "world"
        else:
            self.frame_to_body[frame.name] = int(body_id)
            self.frame_coordinates[frame.name] = coordinates
        self.reindex()

    def delete_frame(self, name: str) -> bool:
        existed = name in self.frames or name in self.frame_to_body
        self.frames.pop(name, None)
        self.frame_to_body.pop(name, None)
        self.frame_coordinates.pop(name, None)
        self.reindex()
        return existed

    def add_force(self, force: Force) -> None:
        self.forces[force.name] = force
        self.reindex()

    def delete_force(self, name: str) -> bool:
        if name not in self.forces:
            return False
        del self.forces[name]
        self.reindex()
        return True

    def add_torque(self, torque: Torque) -> None:
        self.torques[torque.name] = torque
        self.reindex()

    def delete_torque(self, name: str) -> bool:
        if name not in self.torques:
            return False
        del self.torques[name]
        self.reindex()
        return True

    def delete_bodies(self, body_ids: Iterable[int]) -> ChangeSet:
        """Remove bodies and everything that hangs off them.

        Single-body and multi-body deletion both use this path. Motors are
        stored on joints, so removing a joint removes its motor.
        """
        wanted = {int(body_id) for body_id in body_ids}
        deleted_joints = [
            name for name, joint in self.joints.items()
            if int(joint.body1_id) in wanted or int(joint.body2_id) in wanted
        ]
        for name in deleted_joints:
            del self.joints[name]
        deleted_frames = [
            name for name, parent in self.frame_to_body.items() if int(parent) in wanted
        ]
        for name in deleted_frames:
            self.frames.pop(name, None)
            self.frame_coordinates.pop(name, None)
            del self.frame_to_body[name]
        deleted_forces = [
            name for name, force in self.forces.items() if int(force.body_id) in wanted
        ]
        for name in deleted_forces:
            del self.forces[name]
        deleted_torques = [
            name for name, torque in self.torques.items() if int(torque.body_id) in wanted
        ]
        for name in deleted_torques:
            del self.torques[name]
        if self.state is not None:
            for body_id in wanted:
                self.state.remove_body_pose(body_id)
        self.bodies[:] = [body for body in self.bodies if int(body.id) not in wanted]
        if deleted_joints or wanted:
            self.note_topology()
        self.reindex()
        return ChangeSet(
            deleted_body_ids=sorted(wanted),
            deleted_joints=deleted_joints,
            deleted_frames=deleted_frames,
            deleted_forces=deleted_forces,
            deleted_torques=deleted_torques,
        )

    def reindex(self) -> None:
        self.joints_by_body = {}
        for name, joint in self.joints.items():
            for body_id in (int(joint.body1_id), int(joint.body2_id)):
                self.joints_by_body.setdefault(body_id, []).append(name)
        self.frames_by_body = {}
        for name, body_id in self.frame_to_body.items():
            self.frames_by_body.setdefault(int(body_id), []).append(name)
        self.forces_by_body = {}
        for name, force in self.forces.items():
            self.forces_by_body.setdefault(int(force.body_id), []).append(name)
        self.torques_by_body = {}
        for name, torque in self.torques.items():
            self.torques_by_body.setdefault(int(torque.body_id), []).append(name)
