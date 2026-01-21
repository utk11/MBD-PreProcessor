"""
Motor renderer for visualizing motor actuation on joints
"""
from typing import Dict, Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder
from OCC.Core.Geom import Geom_Line
from OCC.Core.Graphic3d import Graphic3d_NOM_PLASTIC
from OCC.Display.OCCViewer import Viewer3d
from core.data_structures import Joint, MotorType, JointType, RigidBody
from typing import List
import numpy as np


class MotorRenderer:
    """Handles motor visualization as indicators on joints in the 3D viewer"""
    
    # Motor colors based on type
    VELOCITY_COLOR = Quantity_Color(0.0, 0.8, 0.0, Quantity_TOC_RGB)  # Green
    TORQUE_COLOR = Quantity_Color(1.0, 0.5, 0.0, Quantity_TOC_RGB)    # Orange
    POSITION_COLOR = Quantity_Color(0.6, 0.0, 1.0, Quantity_TOC_RGB)  # Purple
    
    def __init__(self, display):
        """
        Initialize the motor renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.motor_shapes: Dict[str, list] = {}  # joint_name -> [ais_shapes]
    
    def render_motor(self, joint: Joint, bodies: List[RigidBody], ground_body: RigidBody, visible: bool = True):
        """
        Render a motor indicator on a joint
        
        Args:
            joint: Joint object with motor to render
            bodies: List of rigid bodies
            ground_body: Ground body reference
            visible: Whether to make the motor visible immediately
        """
        if not joint.is_motorized:
            return
        
        # Remove existing visualization if any
        if joint.name in self.motor_shapes:
            self.remove_motor(joint.name)
        
        # Get joint location (use frame1 in world coordinates)
        body1 = ground_body if joint.body1_id == -1 else next((b for b in bodies if b.id == joint.body1_id), None)
        
        if not body1 or not joint.frame1:
            print(f"Warning: Cannot render motor for joint {joint.name} - missing body or frame")
            return
        
        # Transform frame1 to world coordinates
        if body1.local_frame and joint.body1_id != -1:
            world_origin = body1.local_frame.origin + body1.local_frame.rotation_matrix @ joint.frame1.origin
            world_rotation = body1.local_frame.rotation_matrix @ joint.frame1.rotation_matrix
        else:
            world_origin = joint.frame1.origin
            world_rotation = joint.frame1.rotation_matrix
        
        # Get joint axis in world coordinates
        axis_map = {
            "+X": np.array([1, 0, 0]),
            "-X": np.array([-1, 0, 0]),
            "+Y": np.array([0, 1, 0]),
            "-Y": np.array([0, -1, 0]),
            "+Z": np.array([0, 0, 1]),
            "-Z": np.array([0, 0, -1])
        }
        local_axis = axis_map.get(joint.axis, np.array([0, 0, 1]))
        world_axis = world_rotation @ local_axis
        
        # Create motor visualization based on type
        shapes = []
        
        if joint.motor_type == MotorType.VELOCITY:
            # Curved arrow for velocity (like rotation/translation symbol)
            shapes = self._create_velocity_indicator(world_origin, world_axis, joint.motor_value > 0)
            color = self.VELOCITY_COLOR
        
        elif joint.motor_type == MotorType.TORQUE:
            # Straight arrow for torque/force
            shapes = self._create_torque_indicator(world_origin, world_axis, joint.motor_value > 0)
            color = self.TORQUE_COLOR
        
        elif joint.motor_type == MotorType.POSITION:
            # Target marker for position control
            shapes = self._create_position_indicator(world_origin, world_axis)
            color = self.POSITION_COLOR
        
        # Display shapes
        ais_shapes = []
        for shape in shapes:
            ais = AIS_Shape(shape)
            ais.SetColor(color)
            ais.SetMaterial(Graphic3d_NOM_PLASTIC)
            self.display.Context.Display(ais, True)
            
            if not visible:
                self.display.Context.Erase(ais, True)
            
            ais_shapes.append(ais)
        
        self.motor_shapes[joint.name] = ais_shapes
        
        print(f"Rendered motor for joint '{joint.name}' at {world_origin}")
    
    def _create_velocity_indicator(self, origin: np.ndarray, axis: np.ndarray, positive: bool) -> list:
        """Create a curved arrow indicator for velocity motors"""
        shapes = []
        
        # Create a cylinder ring to indicate rotation/translation
        radius = 0.15
        height = 0.05
        
        # Create coordinate system perpendicular to axis
        if abs(axis[2]) < 0.9:
            perp = np.cross(axis, np.array([0, 0, 1]))
        else:
            perp = np.cross(axis, np.array([1, 0, 0]))
        perp = perp / np.linalg.norm(perp)
        
        # Create small cylinder/disk at joint
        axis_dir = gp_Dir(axis[0], axis[1], axis[2])
        origin_pnt = gp_Pnt(origin[0], origin[1], origin[2])
        cyl_axis = gp_Ax2(origin_pnt, axis_dir)
        
        cylinder = BRepPrimAPI_MakeCylinder(cyl_axis, radius * 0.5, height).Shape()
        shapes.append(cylinder)
        
        # Add arrow head to indicate direction
        arrow_offset = radius * perp
        if not positive:
            arrow_offset = -arrow_offset
        
        arrow_base = origin + arrow_offset
        arrow_tip = arrow_base + perp * 0.1 * (1 if positive else -1)
        
        cone_height = 0.08
        cone_radius = 0.04
        cone_tip_pnt = gp_Pnt(arrow_tip[0], arrow_tip[1], arrow_tip[2])
        cone_dir = gp_Dir(perp[0] * (1 if positive else -1), 
                          perp[1] * (1 if positive else -1), 
                          perp[2] * (1 if positive else -1))
        cone_base_pnt = gp_Pnt((arrow_tip - cone_dir.XYZ().X() * cone_height)[0],
                               (arrow_tip - cone_dir.XYZ().Y() * cone_height)[1],
                               (arrow_tip - cone_dir.XYZ().Z() * cone_height)[2])
        cone_axis = gp_Ax2(cone_base_pnt, cone_dir)
        
        cone = BRepPrimAPI_MakeCone(cone_axis, cone_radius, 0.0, cone_height).Shape()
        shapes.append(cone)
        
        return shapes
    
    def _create_torque_indicator(self, origin: np.ndarray, axis: np.ndarray, positive: bool) -> list:
        """Create a straight arrow indicator for torque/force motors"""
        shapes = []
        
        # Create arrow along the axis
        arrow_length = 0.3
        direction = axis if positive else -axis
        
        end_point = origin + direction * arrow_length
        
        # Arrow shaft
        start_pnt = gp_Pnt(origin[0], origin[1], origin[2])
        end_pnt = gp_Pnt(end_point[0], end_point[1], end_point[2])
        edge = BRepBuilderAPI_MakeEdge(start_pnt, end_pnt).Edge()
        shapes.append(edge)
        
        # Arrow head
        cone_height = arrow_length * 0.2
        cone_radius = arrow_length * 0.08
        
        cone_base_center = end_point - direction * cone_height
        axis_dir = gp_Dir(direction[0], direction[1], direction[2])
        cone_base_pnt = gp_Pnt(cone_base_center[0], cone_base_center[1], cone_base_center[2])
        cone_axis = gp_Ax2(cone_base_pnt, axis_dir)
        
        cone = BRepPrimAPI_MakeCone(cone_axis, cone_radius, 0.0, cone_height).Shape()
        shapes.append(cone)
        
        return shapes
    
    def _create_position_indicator(self, origin: np.ndarray, axis: np.ndarray) -> list:
        """Create a target marker indicator for position motors"""
        shapes = []
        
        # Create crosshair marker
        size = 0.2
        
        # Get perpendicular vectors
        if abs(axis[2]) < 0.9:
            perp1 = np.cross(axis, np.array([0, 0, 1]))
        else:
            perp1 = np.cross(axis, np.array([1, 0, 0]))
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(axis, perp1)
        perp2 = perp2 / np.linalg.norm(perp2)
        
        # Create cross lines
        for perp in [perp1, perp2]:
            start = origin - perp * size
            end = origin + perp * size
            
            start_pnt = gp_Pnt(start[0], start[1], start[2])
            end_pnt = gp_Pnt(end[0], end[1], end[2])
            edge = BRepBuilderAPI_MakeEdge(start_pnt, end_pnt).Edge()
            shapes.append(edge)
        
        return shapes
    
    def remove_motor(self, joint_name: str):
        """
        Remove motor visualization
        
        Args:
            joint_name: Name of the joint to remove motor visualization from
        """
        if joint_name in self.motor_shapes:
            for ais in self.motor_shapes[joint_name]:
                self.display.Context.Remove(ais, True)
            del self.motor_shapes[joint_name]
    
    def update_motor(self, joint: Joint, bodies: List[RigidBody], ground_body: RigidBody):
        """
        Update motor visualization (re-render)
        
        Args:
            joint: Joint with updated motor
            bodies: List of rigid bodies
            ground_body: Ground body reference
        """
        self.remove_motor(joint.name)
        if joint.is_motorized:
            self.render_motor(joint, bodies, ground_body)
    
    def set_motor_visibility(self, joint_name: str, visible: bool):
        """
        Set visibility of motor visualization
        
        Args:
            joint_name: Name of the joint
            visible: True to show, False to hide
        """
        if joint_name in self.motor_shapes:
            for ais in self.motor_shapes[joint_name]:
                if visible:
                    self.display.Context.Display(ais, True)
                else:
                    self.display.Context.Erase(ais, True)
    
    def clear_all(self):
        """Remove all motor visualizations"""
        for joint_name in list(self.motor_shapes.keys()):
            self.remove_motor(joint_name)
