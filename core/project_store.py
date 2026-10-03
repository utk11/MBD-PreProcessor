"""Versioned project files.

Schema 2 stores body poses, reference frames, local joint markers, attachment
records, motors, forces, torques, body settings, and the source CAD identity.
Schema 1 (the historical ``version: "1.0"`` file) is still read. Fields that
were never saved use documented defaults. Markers are captured only after the
imported poses exist. Attachment of a user frame is not guessed.

Saving writes a temporary file in the destination directory and replaces the
project file only after that write succeeds. A failed load does not publish a
document; the caller keeps the current one.
"""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from core.data_structures import (
    Force,
    Frame,
    Joint,
    JointType,
    MotorType,
    Torque,
    State,
)
from core.assembly_document import AssemblyDocument
from core.kinematics import capture_joint_markers


PROJECT_SCHEMA_VERSION = 2

V1_MIGRATION_NOTES = (
    "This project was saved as version 1. Body poses were not stored, so the imported CAD placement is used.",
    "Joint markers were not stored. They are captured from the imported poses, not from a later dragged pose.",
    "Motors, forces, torques, and body visibility settings were not stored and use their defaults.",
    "User frames have no recorded parent body. They are restored as world frames and are not attached to a body.",
)


class ProjectValidationError(ValueError):
    """The file is not a usable project. The open document must stay as it is."""


@dataclass
class LoadedProject:
    schema_version: int
    step_file: str
    step_file_relative: Optional[str]
    step_fingerprint: Optional[str]
    unit_scale: Optional[float]
    data: dict
    notes: List[str] = field(default_factory=list)


def fingerprint_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def relative_cad_path(project_path: str, step_path: str) -> Optional[str]:
    """A path relative to the project file, when both are on the same drive."""
    try:
        relative = os.path.relpath(os.path.abspath(step_path), os.path.dirname(os.path.abspath(project_path)))
    except ValueError:
        return None
    return relative.replace("\\", "/")


def resolve_step_file(project_path: str, loaded: LoadedProject) -> Optional[str]:
    """Return an existing CAD path, preferring the relative path when it resolves."""
    project_dir = os.path.dirname(os.path.abspath(project_path))
    candidates = []
    if loaded.step_file_relative:
        candidates.append(os.path.normpath(os.path.join(project_dir, loaded.step_file_relative)))
    if loaded.step_file:
        candidates.append(loaded.step_file)
        candidates.append(os.path.normpath(os.path.join(project_dir, os.path.basename(loaded.step_file))))
    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)
        if os.path.isfile(candidate):
            return candidate
    return None


def read_project(path: str) -> LoadedProject:
    """Read and validate a project file. Does not import geometry or replace a document."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise ProjectValidationError(f"Project file is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ProjectValidationError("Project file must contain a JSON object.")
    schema = _schema_of(data)
    notes: List[str] = []
    if schema == 1:
        notes.extend(V1_MIGRATION_NOTES)
    step_file = data.get("step_file") or ""
    if schema == 2 and not step_file and not data.get("step_file_relative"):
        raise ProjectValidationError("Project file does not name a STEP file.")
    if schema == 1 and not step_file:
        raise ProjectValidationError("Version 1 project file does not name a STEP file.")
    _validate_collections(data, schema)
    unit_scale = data.get("unit_scale")
    if unit_scale is not None and not isinstance(unit_scale, (int, float)):
        raise ProjectValidationError("unit_scale must be a number.")
    return LoadedProject(
        schema_version=schema,
        step_file=str(step_file),
        step_file_relative=data.get("step_file_relative"),
        step_fingerprint=data.get("step_fingerprint"),
        unit_scale=None if unit_scale is None else float(unit_scale),
        data=data,
        notes=notes,
    )


def validate_against_bodies(loaded: LoadedProject, body_ids: List[int]) -> None:
    """Reject references to bodies that the imported CAD file does not contain."""
    known = {int(body_id) for body_id in body_ids}
    if loaded.schema_version >= 2:
        saved = {int(entry["id"]) for entry in loaded.data["bodies"]}
        if not saved.issubset(known):
            raise ProjectValidationError(f"Project references bodies missing from the CAD file: {sorted(saved - known)}.")
        # References must belong to the saved assembly, not merely the source
        # CAD: omitted bodies were deleted by the user.
        known = saved
    known.add(-1)
    for joint in loaded.data.get("joints", []):
        for key in ("body1_id", "body2_id"):
            if int(joint[key]) not in known:
                raise ProjectValidationError(
                    f"Joint '{joint.get('name', '?')}' references missing body {joint[key]}."
                )
    for entry in loaded.data.get("bodies", []):
        if int(entry.get("id")) not in known:
            raise ProjectValidationError(f"Project body {entry.get('id')} is not in the CAD file.")
    for entry in list(loaded.data.get("forces", [])) + list(loaded.data.get("torques", [])):
        if int(entry.get("body_id")) not in known:
            raise ProjectValidationError(
                f"Load '{entry.get('name', '?')}' references missing body {entry.get('body_id')}."
            )
    for frame in loaded.data.get("frames", []):
        parent = frame.get("parent_body_id")
        if parent is not None and int(parent) not in known:
            raise ProjectValidationError(
                f"Frame '{frame.get('name', '?')}' references missing body {parent}."
            )


def fingerprint_note(loaded: LoadedProject, step_path: str) -> Optional[str]:
    if not loaded.step_fingerprint:
        return None
    current = fingerprint_file(step_path)
    if current != loaded.step_fingerprint:
        return (
            "The STEP file fingerprint does not match the one stored in the project. "
            "The file may have changed since the project was saved."
        )
    return None


def prepare_import_document(bodies, unit_scale: float, step_path: str,
                            loaded: Optional[LoadedProject] = None):
    """Restore into an unpublished document, including all fallible steps.

    Return the candidate, migration notes, and imported poses used as the
    rendering baseline. The caller publishes only after this function succeeds.
    """
    staged = AssemblyDocument()
    staged.replace_bodies(bodies)
    staged.unit_scale = float(unit_scale)
    staged.source_cad_path = step_path
    staged.state = State()
    imported_poses = {}
    for body in staged.bodies:
        origin = np.zeros(3) if body.local_frame is None else body.local_frame.origin
        rotation = np.eye(3) if body.local_frame is None else body.local_frame.rotation_matrix
        staged.state.set_body_pose(body.id, origin, rotation)
        imported_poses[int(body.id)] = (np.array(origin, copy=True), np.array(rotation, copy=True))
        body.state = staged.state
    notes = []
    if loaded is not None:
        try:
            _validate_collections(loaded.data, loaded.schema_version)
            validate_against_bodies(loaded, [body.id for body in staged.bodies])
            notes = apply_project(staged, loaded)
            note = fingerprint_note(loaded, step_path)
            if note:
                notes.append(note)
        except (ValueError, TypeError, KeyError, OSError) as exc:
            raise ProjectValidationError(f"Could not restore project: {exc}") from exc
    return staged, notes, imported_poses


def save_project(path: str, document, step_file: str) -> None:
    """Write schema 2 atomically. The destination is replaced only after a full write."""
    payload = build_payload(document, path, step_file)
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".mbdp-", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except Exception:
        if os.path.exists(temporary):
            try:
                os.remove(temporary)
            except OSError:
                pass
        raise


def build_payload(document, project_path: str, step_file: str) -> dict:
    absolute = os.path.abspath(step_file)
    bodies = []
    for body in document.bodies:
        entry = {
            "id": int(body.id),
            "name": body.name,
            "visible": bool(body.visible),
            "contact_enabled": bool(body.contact_enabled),
        }
        if document.state is not None:
            pose = document.state.get_body_pose(body.id)
            if pose is not None:
                entry["pose"] = _matrix_record(pose.origin, pose.rotation_matrix)
        if body.local_frame is not None:
            entry["reference_frame"] = _frame_record(body.local_frame)
        bodies.append(entry)
    frames = []
    for name, frame in document.frames.items():
        parent = document.frame_to_body.get(name)
        frames.append({
            "name": frame.name,
            "origin": _vec(frame.origin),
            "rotation_matrix": _mat(frame.rotation_matrix),
            "parent_body_id": None if parent is None else int(parent),
            "coordinates": document.frame_coordinates.get(name, "reference_geometry" if parent is not None else "world"),
        })
    joints = []
    for joint in document.joints.values():
        entry = {
            "name": joint.name,
            "type": joint.joint_type.name,
            "body1_id": int(joint.body1_id),
            "body2_id": int(joint.body2_id),
            "axis": joint.axis,
            "frame_name": joint.frame.name if joint.frame is not None else joint.name,
            "frame_origin": _vec(joint.frame.origin) if joint.frame is not None else [0.0, 0.0, 0.0],
            "frame_rotation": _mat(joint.frame.rotation_matrix) if joint.frame is not None else _mat(np.eye(3)),
            "is_motorized": bool(joint.is_motorized),
            "motor_type": joint.motor_type.name if joint.is_motorized and joint.motor_type is not None else None,
            "motor_value": float(joint.motor_value) if joint.is_motorized else 0.0,
        }
        if joint.marker1 is not None:
            entry["marker1"] = _frame_record(joint.marker1)
        if joint.marker1_source is not None:
            entry["marker1_source"] = str(joint.marker1_source)
        if joint.marker1_axis is not None:
            entry["marker1_axis"] = str(joint.marker1_axis)
        if joint.marker1_flip:
            entry["marker1_flip"] = True
        if joint.marker2 is not None:
            entry["marker2"] = _frame_record(joint.marker2)
        if joint.marker2_source is not None:
            entry["marker2_source"] = str(joint.marker2_source)
        if joint.marker2_axis is not None:
            entry["marker2_axis"] = str(joint.marker2_axis)
        if joint.marker2_flip:
            entry["marker2_flip"] = True
        joints.append(entry)
    return {
        "schema_version": PROJECT_SCHEMA_VERSION,
        "version": "2.0",
        "step_file": absolute,
        "step_file_relative": relative_cad_path(project_path, absolute),
        "step_fingerprint": fingerprint_file(absolute) if os.path.isfile(absolute) else None,
        "unit_scale": float(document.unit_scale),
        "bodies": bodies,
        "frames": frames,
        "joints": joints,
        "forces": [_force_record(force) for force in document.forces.values()],
        "torques": [_torque_record(torque) for torque in document.torques.values()],
    }


def apply_project(document, loaded: LoadedProject) -> List[str]:
    """Restore relationships onto a document whose bodies and reference poses already exist.

    Call this only after geometry import has installed bodies and the initial
    ``State``. Version 1 leaves those imported poses in place.
    """
    notes = list(loaded.notes)
    data = loaded.data
    by_id = {int(body.id): body for body in document.bodies}
    if loaded.schema_version >= 2:
        validate_against_bodies(loaded, list(by_id))
        saved_ids = {int(entry["id"]) for entry in data["bodies"]}
        document.delete_bodies(set(by_id) - saved_ids)
        by_id = {int(body.id): body for body in document.bodies}
        for entry in data.get("bodies", []):
            body = by_id.get(int(entry["id"]))
            if body is None:
                continue
            body.visible = bool(entry.get("visible", True))
            body.contact_enabled = bool(entry.get("contact_enabled", True))
            reference = entry.get("reference_frame")
            if reference is not None and body.local_frame is not None:
                body.local_frame.origin = np.array(reference["origin"], dtype=float)
                body.local_frame.rotation_matrix = np.array(reference["rotation_matrix"], dtype=float)
            pose = entry.get("pose")
            if pose is not None and document.state is not None:
                document.state.set_body_pose(
                    body.id,
                    np.array(pose["origin"], dtype=float),
                    np.array(pose["rotation_matrix"], dtype=float),
                )
    document.frames.clear()
    document.frame_to_body.clear()
    document.frame_coordinates.clear()
    for entry in data.get("frames", []):
        frame = Frame(
            name=entry["name"],
            origin=np.array(entry["origin"], dtype=float),
            rotation_matrix=np.array(entry["rotation_matrix"], dtype=float),
        )
        parent = entry.get("parent_body_id")
        coordinates = entry.get("coordinates")
        if loaded.schema_version == 1 or parent is None:
            coordinates = "world"
            parent = None
        elif coordinates not in ("world", "reference_geometry", "body_local"):
            raise ProjectValidationError(
                f"Frame '{frame.name}' has an unknown coordinate convention '{coordinates}'."
            )
        document.add_frame(frame, None if parent is None else int(parent), coordinates or "world")
    document.joints.clear()
    document.forces.clear()
    document.torques.clear()
    for entry in data.get("joints", []):
        frame = Frame(
            name=entry.get("frame_name", entry["name"]),
            origin=np.array(entry.get("frame_origin", [0.0, 0.0, 0.0]), dtype=float),
            rotation_matrix=np.array(entry.get("frame_rotation", np.eye(3).tolist()), dtype=float),
        )
        joint = Joint(
            name=entry["name"],
            joint_type=JointType[entry["type"]],
            body1_id=int(entry["body1_id"]),
            body2_id=int(entry["body2_id"]),
            frame=frame,
            axis=entry.get("axis", "+Z"),
        )
        marker1 = entry.get("marker1")
        marker2 = entry.get("marker2")
        if marker1 is not None and marker2 is not None:
            joint.marker1 = _frame_from_record(marker1, f"{joint.name}_marker1")
            joint.marker2 = _frame_from_record(marker2, f"{joint.name}_marker2")
            joint.marker1_source = entry.get("marker1_source")
            joint.marker2_source = entry.get("marker2_source")
            joint.marker1_axis = entry.get("marker1_axis")
            joint.marker2_axis = entry.get("marker2_axis")
            joint.marker1_flip = bool(entry.get("marker1_flip", False))
            joint.marker2_flip = bool(entry.get("marker2_flip", False))
        elif document.state is not None:
            capture_joint_markers(
                joint,
                _endpoint_pose(document, joint.body1_id),
                _endpoint_pose(document, joint.body2_id),
            )
        if entry.get("is_motorized") and entry.get("motor_type"):
            joint.add_motor(MotorType[entry["motor_type"]], float(entry.get("motor_value", 0.0)))
        document.joints[joint.name] = joint
    for entry in data.get("forces", []):
        document.forces[entry["name"]] = Force(
            name=entry["name"],
            body_id=int(entry["body_id"]),
            frame=_frame_from_record(entry["frame"], entry["name"]),
            magnitude=float(entry["magnitude"]),
            direction=np.array(entry["direction"], dtype=float),
        )
    for entry in data.get("torques", []):
        document.torques[entry["name"]] = Torque(
            name=entry["name"],
            body_id=int(entry["body_id"]),
            frame=_frame_from_record(entry["frame"], entry["name"]),
            magnitude=float(entry["magnitude"]),
            axis=np.array(entry["axis"], dtype=float),
        )
    document.note_topology()
    document.note_markers()
    document.note_external_pose()
    document.reindex()
    return notes


def _endpoint_pose(document, body_id: int):
    if int(body_id) == -1 or document.state is None:
        return np.zeros(3), np.eye(3)
    pose = document.state.get_body_pose(int(body_id))
    if pose is None:
        return np.zeros(3), np.eye(3)
    return pose.origin.copy(), pose.rotation_matrix.copy()


def _schema_of(data: dict) -> int:
    schema = data.get("schema_version")
    legacy = str(data.get("version", ""))
    if schema is None and legacy == "1.0":
        return 1
    if schema == 1 or legacy == "1.0":
        return 1
    if schema == PROJECT_SCHEMA_VERSION or legacy in ("2.0", "2"):
        return PROJECT_SCHEMA_VERSION
    raise ProjectValidationError(
        f"Unsupported project schema {schema!r} / version {legacy!r}."
    )


def _validate_collections(data: dict, schema: int) -> None:
    for collection in ("joints", "frames", "bodies", "forces", "torques"):
        entries = data.get(collection, [])
        if not isinstance(entries, list) or any(not isinstance(entry, dict) for entry in entries):
            raise ProjectValidationError(f"{collection} must be a list of records.")
    if schema >= 2 and "bodies" not in data:
        raise ProjectValidationError("Version 2 project must record its body membership.")
    joints = data.get("joints", [])
    frames = data.get("frames", [])
    if not isinstance(joints, list) or not isinstance(frames, list):
        raise ProjectValidationError("joints and frames must be lists.")
    names = set()
    for joint in joints:
        name = joint.get("name")
        if not name or name in names:
            raise ProjectValidationError(f"Joint name {name!r} is missing or duplicated.")
        names.add(name)
        if "body1_id" not in joint or "body2_id" not in joint or "type" not in joint:
            raise ProjectValidationError(f"Joint '{name}' is missing type or body ids.")
        try:
            JointType[joint["type"]]
        except KeyError as exc:
            raise ProjectValidationError(f"Joint '{name}' has unknown type {joint['type']!r}.") from exc
        if schema >= 2:
            _validate_optional_frame(joint.get("marker1"), f"{name} marker1")
            _validate_optional_frame(joint.get("marker2"), f"{name} marker2")
            for side in ("marker1", "marker2"):
                source = joint.get(f"{side}_source")
                axis = joint.get(f"{side}_axis")
                flip = joint.get(f"{side}_flip", False)
                if source is not None and not isinstance(source, str):
                    raise ProjectValidationError(f"Joint '{name}' {side} source must be text.")
                if axis is not None and axis not in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
                    raise ProjectValidationError(f"Joint '{name}' has an invalid {side} axis.")
                if not isinstance(flip, bool):
                    raise ProjectValidationError(f"Joint '{name}' {side} flip must be a boolean.")
            if (joint.get("marker1") is None) != (joint.get("marker2") is None):
                raise ProjectValidationError(f"Joint '{name}' must record both marker frames or neither.")
            if joint.get("is_motorized") and joint.get("motor_type") not in (None, "VELOCITY", "TORQUE", "POSITION"):
                raise ProjectValidationError(f"Joint '{name}' has an unknown motor type.")
    frame_names = set()
    for frame in frames:
        name = frame.get("name")
        if not name or name in frame_names:
            raise ProjectValidationError(f"Frame name {name!r} is missing or duplicated.")
        frame_names.add(name)
        _require_pose(frame, f"frame '{name}'")
        if schema >= 2 and frame.get("coordinates", "world") not in ("world", "reference_geometry", "body_local"):
            raise ProjectValidationError(f"Frame '{name}' has an unknown coordinate convention.")
    if schema >= 2:
        body_ids = set()
        for entry in data.get("bodies", []):
            if "id" not in entry:
                raise ProjectValidationError("A body record is missing its id.")
            try:
                body_id = int(entry["id"])
            except (ValueError, TypeError) as exc:
                raise ProjectValidationError("Body id must be an integer.") from exc
            if body_id in body_ids:
                raise ProjectValidationError(f"Body id {body_id} is duplicated.")
            body_ids.add(body_id)
            if "pose" in entry:
                _require_pose(entry["pose"], f"body {entry['id']} pose")
            if "reference_frame" in entry and entry["reference_frame"] is not None:
                _require_pose(entry["reference_frame"], f"body {entry['id']} reference frame")
        for entry in data.get("forces", []):
            _require_load(entry, "force")
        for entry in data.get("torques", []):
            _require_load(entry, "torque", axis_key="axis")


def _require_load(entry: dict, kind: str, axis_key: str = "direction") -> None:
    if "name" not in entry or "body_id" not in entry or "magnitude" not in entry or axis_key not in entry:
        raise ProjectValidationError(f"A {kind} record is incomplete.")
    if "frame" not in entry:
        raise ProjectValidationError(f"{kind.capitalize()} '{entry['name']}' has no frame.")
    _require_pose(entry["frame"], f"{kind} '{entry['name']}' frame")


def _validate_optional_frame(record, label: str) -> None:
    if record is not None:
        _require_pose(record, label)


def _require_pose(record: dict, label: str) -> None:
    if not isinstance(record, dict) or "origin" not in record or "rotation_matrix" not in record:
        raise ProjectValidationError(f"{label} needs an origin and a rotation matrix.")
    origin = record["origin"]
    rotation = record["rotation_matrix"]
    if not isinstance(origin, list) or len(origin) != 3:
        raise ProjectValidationError(f"{label} origin must have 3 components.")
    if not isinstance(rotation, list) or len(rotation) != 3 or any(not isinstance(row, list) or len(row) != 3 for row in rotation):
        raise ProjectValidationError(f"{label} rotation must be a 3x3 matrix.")
    try:
        finite = np.isfinite(np.asarray(origin, dtype=float)).all() and np.isfinite(np.asarray(rotation, dtype=float)).all()
    except (ValueError, TypeError) as exc:
        raise ProjectValidationError(f"{label} must contain numeric components.") from exc
    if not finite:
        raise ProjectValidationError(f"{label} must contain finite components.")


def _vec(values) -> List[float]:
    return [float(value) for value in np.asarray(values, dtype=float).reshape(3)]


def _mat(values) -> List[List[float]]:
    array = np.asarray(values, dtype=float).reshape(3, 3)
    return [[float(value) for value in row] for row in array]


def _matrix_record(origin, rotation) -> dict:
    return {"origin": _vec(origin), "rotation_matrix": _mat(rotation)}


def _frame_record(frame: Frame) -> dict:
    return {"name": frame.name, **_matrix_record(frame.origin, frame.rotation_matrix)}


def _frame_from_record(record: dict, fallback_name: str) -> Frame:
    return Frame(
        name=record.get("name", fallback_name),
        origin=np.array(record["origin"], dtype=float),
        rotation_matrix=np.array(record["rotation_matrix"], dtype=float),
    )


def _force_record(force: Force) -> dict:
    return {
        "name": force.name,
        "body_id": int(force.body_id),
        "magnitude": float(force.magnitude),
        "direction": _vec(force.direction),
        "frame": _frame_record(force.frame),
    }


def _torque_record(torque: Torque) -> dict:
    return {
        "name": torque.name,
        "body_id": int(torque.body_id),
        "magnitude": float(torque.magnitude),
        "axis": _vec(torque.axis),
        "frame": _frame_record(torque.frame),
    }
