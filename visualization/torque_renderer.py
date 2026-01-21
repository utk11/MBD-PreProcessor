"""
Torque renderer for visualizing applied torques as circular arrows
"""
from typing import Dict, Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir, gp_Circ, gp_Ax1
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Display.OCCViewer import Viewer3d
from core.data_structures import Torque
import numpy as np


class TorqueRenderer:
    """Renderer for visualizing torques as circular arrows"""
    
    # Color constants
    TORQUE_COLOR = Quantity_Color(1.0, 0.0, 1.0, Quantity_TOC_RGB)  # Magenta for torques
    
    def __init__(self, display: Viewer3d):
        """
        Initialize the torque renderer
        
        Args:
            display: The 3D viewer display
        """
        self.display = display
        self.torque_shapes: Dict[str, list] = {}  # torque_name -> [ais_shapes]
        self.torque_scale = 1.0  # Global scale factor
        self.unit_scale = 1.0 # Meters per model unit

    def set_unit_scale(self, scale: float):
        """Set unit scale (Meters per Model Unit)"""
        self.unit_scale = scale
        
    def set_torque_scale(self, scale: float):
        """Set visual scale for torque circular arrows"""
        self.torque_scale = scale
    
    def render_torque(self, torque: Torque, visible: bool = True):
        """
        Render a torque as a circular arrow
        
        Args:
            torque: The Torque object to render
            visible: Whether to display the torque immediately
        """
        # Remove existing visualization if any
        if torque.name in self.torque_shapes:
            self.remove_torque(torque.name)
        
        # Get origin from frame
        # Scale origin from Meters to Model Units
        origin = torque.frame.origin / self.unit_scale
        axis = torque.axis
        
        # Calculate arc radius using logarithmic scaling
        # torque_scale should correspond to ~15% of model size
        base_radius = self.torque_scale
        radius = base_radius * (1.0 + 0.2 * np.log10(max(torque.magnitude, 1.0)))
        
        # Debug output
        print(f"\n=== Torque Rendering Debug ===")
        print(f"Torque: {torque.name}")
        print(f"Origin: {origin}")
        print(f"Axis: {axis}")
        print(f"Magnitude: {torque.magnitude}")
        print(f"Arc radius: {radius}")
        print(f"===========================\n")
        
        # Create a coordinate system with axis as Z
        axis_dir = gp_Dir(axis[0], axis[1], axis[2])
        origin_pnt = gp_Pnt(origin[0], origin[1], origin[2])
        
        # Find perpendicular vector to axis for circle plane
        # Choose a vector not parallel to axis
        if abs(axis[0]) < 0.9:
            perp = np.array([1.0, 0.0, 0.0])
        else:
            perp = np.array([0.0, 1.0, 0.0])
        
        # Make it perpendicular to axis using cross product
        perp = np.cross(axis, perp)
        perp = perp / np.linalg.norm(perp)
        
        # Create circular arc (270 degrees)
        ax2 = gp_Ax2(origin_pnt, axis_dir, gp_Dir(perp[0], perp[1], perp[2]))
        circle = gp_Circ(ax2, radius)
        
        # Create arc from 0 to 270 degrees (3/4 circle)
        arc_maker = GC_MakeArcOfCircle(circle, 0, np.radians(270), True)
        if not arc_maker.IsDone():
            print(f"Warning: Failed to create arc for torque {torque.name}")
            return
        
        arc_curve = arc_maker.Value()
        arc_edge = BRepBuilderAPI_MakeEdge(arc_curve).Edge()
        arc_ais = AIS_Shape(arc_edge)
        
        # Create arrowhead at the end of the arc
        # Calculate end point of arc (at 270 degrees)
        angle = np.radians(270)
        end_on_circle = origin + perp * radius * np.cos(angle) + np.cross(axis, perp) * radius * np.sin(angle)
        
        # Tangent direction at end (perpendicular to radius, in plane of circle)
        tangent = np.cross(axis, (end_on_circle - origin))
        tangent = tangent / np.linalg.norm(tangent)
        
        # Arrow head dimensions
        cone_height = radius * 0.15
        cone_radius = radius * 0.08
        
        # Position cone at end of arc, pointing along tangent
        cone_base_center = end_on_circle - tangent * cone_height
        cone_tip = end_on_circle
        
        # Create cone
        cone_axis_dir = gp_Dir(tangent[0], tangent[1], tangent[2])
        cone_base_pnt = gp_Pnt(cone_base_center[0], cone_base_center[1], cone_base_center[2])
        cone_axis = gp_Ax2(cone_base_pnt, cone_axis_dir)
        
        cone = BRepPrimAPI_MakeCone(cone_axis, cone_radius, 0.0, cone_height).Shape()
        cone_ais = AIS_Shape(cone)
        
        # Set color and display properties
        self.display.Context.SetColor(arc_ais, self.TORQUE_COLOR, False)
        self.display.Context.SetColor(cone_ais, self.TORQUE_COLOR, False)
        
        # Set line width (increased for visibility)
        self.display.Context.SetWidth(arc_ais, 5.0, False)
        
        # Display
        if visible:
            self.display.Context.Display(arc_ais, False)
            self.display.Context.Display(cone_ais, False)
        
        # Store references
        self.torque_shapes[torque.name] = [arc_ais, cone_ais]
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"Torque '{torque.name}' rendered at {origin} with magnitude {torque.magnitude}N·m")
    
    def remove_torque(self, torque_name: str):
        """
        Remove a torque visualization from the viewer
        
        Args:
            torque_name: Name of the torque to remove
        """
        if torque_name in self.torque_shapes:
            for shape in self.torque_shapes[torque_name]:
                self.display.Context.Erase(shape, False)
            del self.torque_shapes[torque_name]
            self.display.Context.UpdateCurrentViewer()
            print(f"Torque '{torque_name}' removed from viewer")
    
    def clear_all_torques(self):
        """Remove all torque visualizations"""
        for torque_name in list(self.torque_shapes.keys()):
            self.remove_torque(torque_name)
        print("All torques cleared from viewer")
    
    def set_torque_scale(self, scale: float):
        """
        Set the global scale factor for torque visualization
        
        Args:
            scale: Scale factor (1.0 = normal size)
        """
        self.torque_scale = scale
        print(f"Torque scale set to {scale}")
