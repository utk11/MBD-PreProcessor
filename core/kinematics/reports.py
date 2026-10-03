"""Typed kinematic solve results.

``SolveReport`` is the public outcome of one request. The trace is an explicit
field. Pose arrays on a report are owned copies; committing them is a separate
step on the GUI thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from core.kinematics.trace import PhaseTrace


@dataclass
class RevisionStamp:
    """Revisions copied onto a request and the result that answers it."""

    document_generation: int = 0
    topology_revision: int = 0
    marker_revision: int = 0
    external_pose_epoch: int = 0
    pose_revision: int = 0
    request_id: int = 0
    gesture_id: int = 0
    epoch: int = 0
    kind: str = ""
    finalize: bool = False
    use_revisions: bool = False


@dataclass
class SolveReport:
    """Outcome of a solve. ``poses`` are owned copies and may be empty."""

    converged: bool
    iterations: int
    final_residual_norm: float
    max_residual: float
    per_joint_residual: Dict[str, float] = field(default_factory=dict)
    moved_bodies: List[int] = field(default_factory=list)
    redundant_joints: List[str] = field(default_factory=list)
    dof: Optional[int] = None
    message: str = ""
    trace: Optional[PhaseTrace] = None
    finite: bool = True
    poses: Dict[int, Tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    target_error: Optional[float] = None
    joint_feasible: Optional[bool] = None
    document_generation: int = 0
    topology_revision: int = 0
    marker_revision: int = 0
    external_pose_epoch: int = 0
    pose_revision: int = 0
    request_id: int = 0
    gesture_id: int = 0
    epoch: int = 0
    kind: str = ""
    finalize: bool = False


def apply_stamp(report: SolveReport, stamp: Optional[RevisionStamp]) -> None:
    """Copy request identity onto a report. Numerical fields are left alone."""
    if stamp is None:
        return
    report.document_generation = int(stamp.document_generation)
    report.topology_revision = int(stamp.topology_revision)
    report.marker_revision = int(stamp.marker_revision)
    report.external_pose_epoch = int(stamp.external_pose_epoch)
    report.pose_revision = int(stamp.pose_revision)
    report.request_id = int(stamp.request_id)
    report.gesture_id = int(stamp.gesture_id)
    report.epoch = int(stamp.epoch)
    report.kind = stamp.kind
    report.finalize = bool(stamp.finalize)


def commit_poses(state, report: SolveReport) -> List[int]:
    """Copy owned pose arrays into ``state``. Returns the body ids written.

    Non-finite reports write nothing. The caller still has to decide whether
    the report's revisions are current.
    """
    if state is None or not report.finite:
        return []
    written: List[int] = []
    for body_id, pair in report.poses.items():
        origin, rotation = pair
        state.set_body_pose(int(body_id), np.asarray(origin, dtype=np.float64),
                            np.asarray(rotation, dtype=np.float64))
        written.append(int(body_id))
    return written
