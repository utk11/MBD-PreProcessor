from typing import List, Optional
from OCC.Core.AIS import AIS_Line, AIS_Shape
from OCC.Core.Geom import Geom_Line, Geom_CartesianPoint
from OCC.Core.Prs3d import Prs3d_LineAspect
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.Aspect import Aspect_TOL_DASH, Aspect_WOL_MEDIUM
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Vec

from core.data_structures import Joint, Frame
from visualization.frame_renderer import FrameRenderer

class JointRenderer:
    def __init__(self, display):
        self.display = display
        self.frame_renderer = FrameRenderer(display)
        self.joint_objects = {}  # Map joint_name -> list of AIS objects (line, frames)

    def render_joint(self, joint: Joint, visible: bool = True):
        """Render a joint visualization (single frame in global coordinates)"""
        
        # If already rendered, remove first
        self.remove_joint(joint.name)
        
        if not visible:
            return

        ais_objects = []

        # Render the joint frame
        # Create proxy frame with unique name for visualization
        frame_proxy = Frame(origin=joint.frame.origin, 
                           rotation_matrix=joint.frame.rotation_matrix, 
                           name=f"Joint_{joint.name}_Frame")
        
        # Render the frame
        self.frame_renderer.render_frame(frame_proxy, visible=True)
        
        # Track the frame so we can remove it later
        ais_objects.append(frame_proxy.name)  # storing name for FrameRenderer cleanup

        self.joint_objects[joint.name] = ais_objects

    def remove_joint(self, joint_name: str):
        """Remove joint visualization"""
        if joint_name in self.joint_objects:
            objects = self.joint_objects[joint_name]
            for obj in objects:
                if isinstance(obj, str):
                    # It's a frame name
                    self.frame_renderer.remove_frame(obj)
                else:
                    # It's an AIS object
                    self.display.Context.Remove(obj, True)
            del self.joint_objects[joint_name]

    def clear(self):
        """Remove all rendered joints"""
        names = list(self.joint_objects.keys())
        for name in names:
            self.remove_joint(name)
