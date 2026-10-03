"""Packed, immutable kinematic model for the CPU experiment.

Python objects stop at this boundary. Evaluators read arrays only: body slots,
joint types, local markers, axes, residual layout, and per-component scatter
columns. Ground always occupies slot 0 so body id ``-1`` is never used as a
NumPy index.

``formulation_version`` ``compat-v1`` freezes the legacy residual order, row
counts, and component membership. ``grouping="unknown"`` is a separate,
explicit policy: two bodies share a component only when a joint couples two
movable unknowns.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, replace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from core.data_structures import Joint, JointType
import core.kinematics.markers as M
from core.kinematics.graph import JointGraph


FORMULATION_COMPAT = "compat-v1"
GROUPING_COMPATIBILITY = "compatibility"
GROUPING_UNKNOWN = "unknown"

TYPE_FIXED = 0
TYPE_REVOLUTE = 1
TYPE_PRISMATIC = 2
TYPE_CYLINDRICAL = 3
TYPE_SPHERICAL = 4

_TYPE_CODE = {
    JointType.FIXED: TYPE_FIXED,
    JointType.REVOLUTE: TYPE_REVOLUTE,
    JointType.PRISMATIC: TYPE_PRISMATIC,
    JointType.CYLINDRICAL: TYPE_CYLINDRICAL,
    JointType.SPHERICAL: TYPE_SPHERICAL,
}
_ROW_COUNT = {
    TYPE_FIXED: 6,
    TYPE_REVOLUTE: 6,
    TYPE_PRISMATIC: 6,
    TYPE_CYLINDRICAL: 6,
    TYPE_SPHERICAL: 3,
}


@dataclass(frozen=True)
class ComponentPlan:
    """One block of residuals and unknown columns solved together."""

    member_ids: Tuple[int, ...]
    joint_indices: np.ndarray
    joint_names: Tuple[str, ...]
    row_offset: np.ndarray
    row_counts: np.ndarray
    n_residual_rows: int
    movable_body_ids: Tuple[int, ...]
    movable_slots: np.ndarray
    scatter_col: np.ndarray
    ncols: int

    def contains(self, body_id: int) -> bool:
        return body_id in self.member_ids or body_id in self.movable_body_ids


@dataclass(frozen=True)
class PreparedModel:
    """Numeric snapshot of topology and constant joint data."""

    formulation_version: str
    grouping: str
    revision: int
    constant_revision: int
    structural_signature: str
    constant_signature: str
    ground_id: int
    body_ids: Tuple[int, ...]
    slot_of: Dict[int, int]
    n_slots: int
    locked_body_ids: Tuple[int, ...]
    joint_names: Tuple[str, ...]
    joint_type: np.ndarray
    slot1: np.ndarray
    slot2: np.ndarray
    body1_id: np.ndarray
    body2_id: np.ndarray
    n_rows: np.ndarray
    row_offset: np.ndarray
    total_rows: int
    marker1_origin: np.ndarray
    marker1_R: np.ndarray
    marker2_origin: np.ndarray
    marker2_R: np.ndarray
    axis_local: np.ndarray
    components: Tuple[ComponentPlan, ...]
    analysis_plan: ComponentPlan
    n_unlocked: int

    @property
    def n_joints(self) -> int:
        return int(self.joint_type.shape[0])


def _as_f64_c(array: np.ndarray) -> np.ndarray:
    out = np.ascontiguousarray(array, dtype=np.float64)
    return out


def _sha256(parts: Iterable[bytes]) -> str:
    digest = hashlib.sha256()
    for part in parts:
        digest.update(part)
    return digest.hexdigest()


def _joint_topology_bytes(joints: Sequence[Joint]) -> bytes:
    chunks = []
    for joint in joints:
        code = _TYPE_CODE.get(joint.joint_type)
        if code is None:
            code = -1
        chunks.append(
            f"{joint.name}|{code}|{int(joint.body1_id)}|{int(joint.body2_id)}".encode("utf-8")
        )
        chunks.append(b"\0")
    return b"".join(chunks)


def _constant_bytes(joints: Sequence[Joint]) -> bytes:
    chunks = [_joint_topology_bytes(joints)]
    for joint in joints:
        if joint.marker1 is None or joint.marker2 is None:
            chunks.append(b"missing-marker\0")
            continue
        axis = M.axis_vector(joint.axis)
        chunks.append(np.asarray(axis, dtype=np.float64).tobytes())
        chunks.append(np.asarray(joint.marker1.origin, dtype=np.float64).tobytes())
        chunks.append(np.asarray(joint.marker1.rotation_matrix, dtype=np.float64).tobytes())
        chunks.append(np.asarray(joint.marker2.origin, dtype=np.float64).tobytes())
        chunks.append(np.asarray(joint.marker2.rotation_matrix, dtype=np.float64).tobytes())
    return b"".join(chunks)


def structural_signature(
    body_ids: Sequence[int],
    joints: Sequence[Joint],
    locked_body_ids: Iterable[int],
    ground_id: int,
    formulation_version: str,
    grouping: str,
) -> str:
    """Topology, locks, formulation, and grouping. Markers and axes are excluded."""
    locked = ",".join(str(int(b)) for b in sorted(set(locked_body_ids)))
    bodies = ",".join(str(int(b)) for b in body_ids)
    header = f"{formulation_version}|{grouping}|{int(ground_id)}|{bodies}|{locked}|".encode("utf-8")
    return _sha256([header, _joint_topology_bytes(joints)])


def constant_signature(
    body_ids: Sequence[int],
    joints: Sequence[Joint],
    locked_body_ids: Iterable[int],
    ground_id: int,
    formulation_version: str,
    grouping: str,
) -> str:
    """Structural signature plus marker frames and joint axes."""
    header = structural_signature(
        body_ids, joints, locked_body_ids, ground_id, formulation_version, grouping
    ).encode("ascii")
    return _sha256([header, _constant_bytes(joints)])


def _validate_joint(joint: Joint) -> int:
    if joint.marker1 is None or joint.marker2 is None:
        raise ValueError(
            f"Joint '{joint.name}' has no local markers; capture them before solving."
        )
    try:
        code = _TYPE_CODE[joint.joint_type]
    except KeyError as exc:
        raise ValueError(f"Unsupported joint type on '{joint.name}': {joint.joint_type}") from exc
    M.axis_vector(joint.axis)
    return code


def _make_plan(
    member_ids: Sequence[int],
    joints: Sequence[Joint],
    joint_indices: Sequence[int],
    movable_body_ids: Sequence[int],
    slot_of: Dict[int, int],
    n_rows: np.ndarray,
    body1_ids: Sequence[int],
    body2_ids: Sequence[int],
) -> ComponentPlan:
    indices = np.ascontiguousarray(np.asarray(list(joint_indices), dtype=np.int32))
    counts = np.ascontiguousarray(n_rows[indices], dtype=np.int32) if indices.size else np.zeros(0, np.int32)
    offset = np.zeros(indices.shape[0], dtype=np.int32)
    if indices.size > 1:
        offset[1:] = np.cumsum(counts[:-1], dtype=np.int32)
    n_residual = int(counts.sum()) if indices.size else 0
    movable = tuple(int(b) for b in movable_body_ids)
    slots = np.ascontiguousarray(
        np.asarray([slot_of[b] for b in movable], dtype=np.int32)
    ) if movable else np.zeros(0, np.int32)
    column_of = {bid: 6 * i for i, bid in enumerate(movable)}
    scatter = np.full((indices.shape[0], 2), -1, dtype=np.int32)
    names = []
    for local, j_index in enumerate(indices.tolist()):
        names.append(joints[j_index].name)
        b1 = int(body1_ids[j_index])
        b2 = int(body2_ids[j_index])
        if b1 in column_of:
            scatter[local, 0] = column_of[b1]
        if b2 in column_of:
            scatter[local, 1] = column_of[b2]
    return ComponentPlan(
        member_ids=tuple(int(b) for b in member_ids),
        joint_indices=indices,
        joint_names=tuple(names),
        row_offset=np.ascontiguousarray(offset),
        row_counts=counts,
        n_residual_rows=n_residual,
        movable_body_ids=movable,
        movable_slots=slots,
        scatter_col=np.ascontiguousarray(scatter),
        ncols=6 * len(movable),
    )


def _compatibility_plans(
    body_ids: Sequence[int],
    joints: Sequence[Joint],
    locked: set,
    ground_id: int,
    slot_of: Dict[int, int],
    n_rows: np.ndarray,
    body1_ids: np.ndarray,
    body2_ids: np.ndarray,
) -> List[ComponentPlan]:
    graph = JointGraph(body_ids, joints, ground_id=ground_id)
    body_set = set(int(b) for b in body_ids)
    plans: List[ComponentPlan] = []
    seen = set()
    index_of = {id(joint): i for i, joint in enumerate(joints)}
    for comp in graph.components():
        key = tuple(sorted(comp))
        if key in seen:
            continue
        seen.add(key)
        comp_joints = [j for j in joints if j.body1_id in comp and j.body2_id in comp]
        if not comp_joints:
            continue
        movable = [b for b in comp if b in body_set and b not in locked]
        plans.append(
            _make_plan(
                comp,
                joints,
                [index_of[id(j)] for j in comp_joints],
                movable,
                slot_of,
                n_rows,
                body1_ids,
                body2_ids,
            )
        )
    return plans


def _unknown_plans(
    body_ids: Sequence[int],
    joints: Sequence[Joint],
    locked: set,
    slot_of: Dict[int, int],
    n_rows: np.ndarray,
    body1_ids: np.ndarray,
    body2_ids: np.ndarray,
) -> List[ComponentPlan]:
    """Union movable bodies only when a joint couples two unknowns."""
    parent: Dict[int, int] = {}
    rank: Dict[int, int] = {}

    def add(body_id: int) -> None:
        if body_id not in parent:
            parent[body_id] = body_id
            rank[body_id] = 0

    def find(body_id: int) -> int:
        root = body_id
        while parent[root] != root:
            root = parent[root]
        while parent[body_id] != root:
            parent[body_id], body_id = root, parent[body_id]
        return root

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra == rb:
            return
        if rank[ra] < rank[rb]:
            ra, rb = rb, ra
        parent[rb] = ra
        if rank[ra] == rank[rb]:
            rank[ra] += 1

    for body_id in body_ids:
        if body_id not in locked:
            add(int(body_id))
    for joint in joints:
        b1, b2 = int(joint.body1_id), int(joint.body2_id)
        if b1 in parent and b2 in parent:
            union(b1, b2)

    plans: List[ComponentPlan] = []
    emitted = set()
    index_of = {id(joint): i for i, joint in enumerate(joints)}
    for body_id in body_ids:
        body_id = int(body_id)
        if body_id not in parent:
            continue
        root = find(body_id)
        if root in emitted:
            continue
        emitted.add(root)
        members = [b for b in body_ids if int(b) in parent and find(int(b)) == root]
        member_set = set(int(b) for b in members)
        comp_joints = []
        for joint in joints:
            b1, b2 = int(joint.body1_id), int(joint.body2_id)
            in1 = b1 in member_set
            in2 = b2 in member_set
            # A joint that touches some other movable body belongs to that
            # body's component. It is an error only when it also touches this one,
            # which would mean the union step missed an unknown-unknown edge.
            if (in1 and b2 in parent and not in2) or (in2 and b1 in parent and not in1):
                raise ValueError(
                    f"Joint '{joint.name}' couples two unknown-components; grouping is inconsistent."
                )
            if in1 or in2:
                comp_joints.append(joint)
        if not comp_joints:
            continue
        plans.append(
            _make_plan(
                members,
                joints,
                [index_of[id(j)] for j in comp_joints],
                members,
                slot_of,
                n_rows,
                body1_ids,
                body2_ids,
            )
        )

    fixed_only = []
    for joint in joints:
        b1, b2 = int(joint.body1_id), int(joint.body2_id)
        if b1 not in parent and b2 not in parent:
            fixed_only.append(joint)
    if fixed_only:
        plans.append(
            _make_plan(
                [],
                joints,
                [index_of[id(j)] for j in fixed_only],
                [],
                slot_of,
                n_rows,
                body1_ids,
                body2_ids,
            )
        )
    return plans


def prepare(
    bodies,
    joints: Sequence[Joint],
    locked_body_ids: Optional[Iterable[int]] = None,
    ground_id: int = -1,
    formulation_version: str = FORMULATION_COMPAT,
    grouping: str = GROUPING_COMPATIBILITY,
    revision: int = 1,
) -> PreparedModel:
    """Pack ``bodies`` and ``joints`` into arrays. Raises ``ValueError`` on bad input."""
    if formulation_version != FORMULATION_COMPAT:
        raise ValueError(
            f"Unsupported formulation_version {formulation_version!r}. "
            f"This build implements {FORMULATION_COMPAT}."
        )
    if grouping not in (GROUPING_COMPATIBILITY, GROUPING_UNKNOWN):
        raise ValueError(f"Unsupported grouping {grouping!r}.")

    body_ids = tuple(int(b.id) for b in bodies)
    if ground_id in body_ids:
        raise ValueError(f"Ground id {ground_id} collides with a body id.")
    if len(set(body_ids)) != len(body_ids):
        raise ValueError("Duplicate body ids.")

    locked = set(int(b) for b in (locked_body_ids or []))
    locked.add(int(ground_id))

    slot_of: Dict[int, int] = {int(ground_id): 0}
    for offset, body_id in enumerate(body_ids):
        slot_of[int(body_id)] = offset + 1
    n_slots = len(body_ids) + 1

    n_joints = len(joints)
    joint_type = np.zeros(n_joints, dtype=np.int32)
    slot1 = np.zeros(n_joints, dtype=np.int32)
    slot2 = np.zeros(n_joints, dtype=np.int32)
    body1_id = np.zeros(n_joints, dtype=np.int32)
    body2_id = np.zeros(n_joints, dtype=np.int32)
    n_rows = np.zeros(n_joints, dtype=np.int32)
    marker1_origin = np.zeros((n_joints, 3), dtype=np.float64)
    marker1_R = np.zeros((n_joints, 3, 3), dtype=np.float64)
    marker2_origin = np.zeros((n_joints, 3), dtype=np.float64)
    marker2_R = np.zeros((n_joints, 3, 3), dtype=np.float64)
    axis_local = np.zeros((n_joints, 3), dtype=np.float64)

    known = set(slot_of)
    for i, joint in enumerate(joints):
        code = _validate_joint(joint)
        b1 = int(joint.body1_id)
        b2 = int(joint.body2_id)
        if b1 not in known or b2 not in known:
            raise ValueError(
                f"Joint '{joint.name}' references unknown body ids {(b1, b2)}."
            )
        joint_type[i] = code
        slot1[i] = slot_of[b1]
        slot2[i] = slot_of[b2]
        body1_id[i] = b1
        body2_id[i] = b2
        n_rows[i] = _ROW_COUNT[code]
        marker1_origin[i] = np.asarray(joint.marker1.origin, dtype=np.float64)
        marker1_R[i] = np.asarray(joint.marker1.rotation_matrix, dtype=np.float64)
        marker2_origin[i] = np.asarray(joint.marker2.origin, dtype=np.float64)
        marker2_R[i] = np.asarray(joint.marker2.rotation_matrix, dtype=np.float64)
        axis_local[i] = M.axis_vector(joint.axis)

    row_offset = np.zeros(n_joints, dtype=np.int32)
    if n_joints > 1:
        row_offset[1:] = np.cumsum(n_rows[:-1], dtype=np.int32)
    total_rows = int(n_rows.sum()) if n_joints else 0
    for label, array in (
        ("marker1_origin", marker1_origin),
        ("marker1_R", marker1_R),
        ("marker2_origin", marker2_origin),
        ("marker2_R", marker2_R),
        ("axis_local", axis_local),
    ):
        if array.size and not np.isfinite(array).all():
            raise ValueError(f"Nonfinite values in {label}.")

    if grouping == GROUPING_COMPATIBILITY:
        components = _compatibility_plans(
            body_ids, joints, locked, ground_id, slot_of, n_rows, body1_id, body2_id
        )
    else:
        components = _unknown_plans(
            body_ids, joints, locked, slot_of, n_rows, body1_id, body2_id
        )

    analysis_movable = [b for b in body_ids if b not in locked]
    analysis_plan = _make_plan(
        body_ids,
        joints,
        list(range(n_joints)),
        analysis_movable,
        slot_of,
        n_rows,
        body1_id,
        body2_id,
    )
    n_unlocked = len(analysis_movable)
    struct_sig = structural_signature(
        body_ids, joints, locked, ground_id, formulation_version, grouping
    )
    const_sig = constant_signature(
        body_ids, joints, locked, ground_id, formulation_version, grouping
    )
    return PreparedModel(
        formulation_version=formulation_version,
        grouping=grouping,
        revision=int(revision),
        constant_revision=1,
        structural_signature=struct_sig,
        constant_signature=const_sig,
        ground_id=int(ground_id),
        body_ids=body_ids,
        slot_of=slot_of,
        n_slots=n_slots,
        locked_body_ids=tuple(sorted(locked)),
        joint_names=tuple(j.name for j in joints),
        joint_type=joint_type,
        slot1=slot1,
        slot2=slot2,
        body1_id=body1_id,
        body2_id=body2_id,
        n_rows=n_rows,
        row_offset=row_offset,
        total_rows=total_rows,
        marker1_origin=_as_f64_c(marker1_origin),
        marker1_R=_as_f64_c(marker1_R),
        marker2_origin=_as_f64_c(marker2_origin),
        marker2_R=_as_f64_c(marker2_R),
        axis_local=_as_f64_c(axis_local),
        components=tuple(components),
        analysis_plan=analysis_plan,
        n_unlocked=n_unlocked,
    )


def refresh_constants(model: PreparedModel, joints: Sequence[Joint]) -> PreparedModel:
    """Replace marker and axis arrays when topology is unchanged."""
    if len(joints) != model.n_joints:
        raise ValueError("Constant refresh requires the same joint count.")
    for i, joint in enumerate(joints):
        code = _validate_joint(joint)
        if code != int(model.joint_type[i]):
            raise ValueError(f"Joint '{joint.name}' changed type; rebuild the model.")
        if int(joint.body1_id) != int(model.body1_id[i]) or int(joint.body2_id) != int(model.body2_id[i]):
            raise ValueError(f"Joint '{joint.name}' changed endpoints; rebuild the model.")
        if joint.name != model.joint_names[i]:
            raise ValueError(f"Joint name changed at index {i}; rebuild the model.")
    marker1_origin = np.array(model.marker1_origin, copy=True)
    marker1_R = np.array(model.marker1_R, copy=True)
    marker2_origin = np.array(model.marker2_origin, copy=True)
    marker2_R = np.array(model.marker2_R, copy=True)
    axis_local = np.array(model.axis_local, copy=True)
    for i, joint in enumerate(joints):
        marker1_origin[i] = np.asarray(joint.marker1.origin, dtype=np.float64)
        marker1_R[i] = np.asarray(joint.marker1.rotation_matrix, dtype=np.float64)
        marker2_origin[i] = np.asarray(joint.marker2.origin, dtype=np.float64)
        marker2_R[i] = np.asarray(joint.marker2.rotation_matrix, dtype=np.float64)
        axis_local[i] = M.axis_vector(joint.axis)
    const_sig = constant_signature(
        model.body_ids,
        joints,
        model.locked_body_ids,
        model.ground_id,
        model.formulation_version,
        model.grouping,
    )
    return replace(
        model,
        constant_revision=model.constant_revision + 1,
        constant_signature=const_sig,
        marker1_origin=_as_f64_c(marker1_origin),
        marker1_R=_as_f64_c(marker1_R),
        marker2_origin=_as_f64_c(marker2_origin),
        marker2_R=_as_f64_c(marker2_R),
        axis_local=_as_f64_c(axis_local),
    )


def component_for_body(model: PreparedModel, body_id: int) -> Optional[ComponentPlan]:
    """Return the plan that owns ``body_id`` as a member or as a joint endpoint."""
    for plan in model.components:
        if plan.contains(body_id):
            return plan
    for plan in model.components:
        for j_index in plan.joint_indices.tolist():
            if int(model.body1_id[j_index]) == body_id or int(model.body2_id[j_index]) == body_id:
                return plan
    return None
