"""Render the two body-attached ends of each joint and their current connector."""

from typing import Dict, Optional

import numpy as np
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.gp import gp_Pnt

from core.data_structures import Frame, Joint
from core.kinematics.markers import marker_world
from core.transforms import body_world_pose
from visualization.frame_renderer import FrameRenderer


class JointRenderer:
    PREVIEW_KEY = "__joint_preview__"

    def __init__(self, display):
        self.display = display
        self.frame_renderer = FrameRenderer(display)
        self.joint_objects: Dict[str, list] = {}

    def render_joint(self, joint: Joint, visible: bool = True, bodies=None, ground_body=None):
        self.remove_joint(joint.name)
        if not visible:
            return

        endpoint1, endpoint2 = self._world_endpoints(joint, bodies or [], ground_body)
        entries = []
        for side, endpoint in enumerate((endpoint1, endpoint2), start=1):
            frame = Frame(
                np.array(endpoint.origin, dtype=float, copy=True),
                np.array(endpoint.rotation_matrix, dtype=float, copy=True),
                f"Joint_{joint.name}_Body{side}",
            )
            self.frame_renderer.render_frame(frame, visible=True)
            if side == 2:
                self.frame_renderer.highlight_frame(frame.name, True)
            entries.append(frame.name)
        connector = self._connector(endpoint1.origin, endpoint2.origin)
        if connector is not None:
            self.display.Context.Display(connector, False)
            entries.append(connector)
        self.joint_objects[joint.name] = entries

    def _world_endpoints(self, joint: Joint, bodies, ground_body):
        if joint.marker1 is None or joint.marker2 is None:
            # Legacy transient visualizations have no captured markers.
            frame = joint.frame
            return (
                Frame(frame.origin.copy(), frame.rotation_matrix.copy(), "marker1"),
                Frame(frame.origin.copy(), frame.rotation_matrix.copy(), "marker2"),
            )
        body_by_id = {int(body.id): body for body in bodies}
        if ground_body is not None:
            body_by_id[-1] = ground_body
        poses = []
        for body_id in (joint.body1_id, joint.body2_id):
            body = body_by_id.get(int(body_id))
            if body is None:
                raise ValueError(f"Joint '{joint.name}' refers to missing body {body_id}.")
            poses.append(body_world_pose(body))
        return (
            marker_world(joint.marker1, *poses[0]),
            marker_world(joint.marker2, *poses[1]),
        )

    def _connector(self, origin1, origin2):
        origin1 = np.asarray(origin1, dtype=float)
        origin2 = np.asarray(origin2, dtype=float)
        if not np.isfinite(origin1).all() or not np.isfinite(origin2).all():
            return None
        scale = self.frame_renderer.unit_scale or 1.0
        p1 = gp_Pnt(*(origin1 / scale).tolist())
        p2 = gp_Pnt(*(origin2 / scale).tolist())
        if p1.Distance(p2) < 1e-9:
            return None
        ais = AIS_Shape(BRepBuilderAPI_MakeEdge(p1, p2).Edge())
        ais.SetColor(Quantity_Color(1.0, 0.8, 0.1, Quantity_TOC_RGB))
        ais.SetWidth(2.0)
        return ais

    def preview_attachments(self, frame1: Frame, frame2: Frame):
        self.clear_preview()
        entries = []
        for side, endpoint in enumerate((frame1, frame2), start=1):
            proxy = Frame(
                np.array(endpoint.origin, dtype=float, copy=True),
                np.array(endpoint.rotation_matrix, dtype=float, copy=True),
                f"Joint_Preview_Body{side}",
            )
            self.frame_renderer.render_frame(proxy, visible=True)
            if side == 2:
                self.frame_renderer.highlight_frame(proxy.name, True)
            entries.append(proxy.name)
        connector = self._connector(frame1.origin, frame2.origin)
        if connector is not None:
            self.display.Context.Display(connector, False)
            entries.append(connector)
        self.joint_objects[self.PREVIEW_KEY] = entries

    def clear_preview(self):
        self.remove_joint(self.PREVIEW_KEY)

    def remove_joint(self, joint_name: str):
        objects = self.joint_objects.pop(joint_name, ())
        for obj in objects:
            if isinstance(obj, str):
                self.frame_renderer.remove_frame(obj)
            else:
                self.display.Context.Remove(obj, False)
        if objects:
            self.display.Context.UpdateCurrentViewer()

    def clear(self):
        for name in list(self.joint_objects):
            self.remove_joint(name)
