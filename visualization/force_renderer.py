"""
Force renderer for visualizing applied forces as arrows
"""
from typing import Dict, Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCC.Core.Geom import Geom_Line
from OCC.Core.Graphic3d import Graphic3d_NOM_PLASTIC
from OCC.Display.OCCViewer import Viewer3d
from core.data_structures import Force
import numpy as np


class ForceRenderer:
    """Handles force visualization as arrows in the 3D viewer"""
    
    FORCE_COLOR = Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB)  # Blue
    ARROW_SCALE = 0.002  # Scale factor for arrow size relative to model
    
    def __init__(self, display):
        """
        Initialize the force renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.force_shapes: Dict[str, list] = {}  # force_name -> [line_ais, cone_ais]
        self.unit_scale = 1.0  # Meters per model unit
        self.force_scale = 1.0  # Visual scale for force arrows
    
    def set_unit_scale(self, scale: float):
        """Set the unit scale (Meters per Model Unit)"""
        self.unit_scale = scale
    
    def set_force_scale(self, scale: float):
        """Set visual scale for force arrows"""
        self.force_scale = scale
    
    def render_force(self, force: Force, visible: bool = True):
        """
        Render a force as an arrow in the 3D viewer
        
        Args:
            force: Force object to render
            visible: Whether to make the force visible immediately
        """
        # Remove existing visualization if any
        if force.name in self.force_shapes:
            self.remove_force(force.name)
        
        # Get origin and direction
        # Scale origin from Meters to Model Units
        origin = force.frame.origin / self.unit_scale
        direction = force.direction
        
        # Scale arrow length based on force magnitude, visual scale, and arrow scale
        # Use logarithmic scaling for better visualization across magnitudes
        # visual_scale (force_scale) should roughly correspond to ~20% of model size
        base_length = self.force_scale 
        length = base_length * (1.0 + 0.2 * np.log10(max(force.magnitude, 1.0)))
        
        # Calculate end point
        end_point = origin + direction * length
        
        # Debug output
        print(f"\n=== Force Rendering Debug ===")
        print(f"Force: {force.name}")
        print(f"Origin: {origin}")
        print(f"Direction: {direction}")
        print(f"Magnitude: {force.magnitude}")
        print(f"Arrow length: {length}")
        print(f"End point: {end_point}")
        print(f"===========================\n")
        
        # Create arrow shaft (line)
        start_pnt = gp_Pnt(origin[0], origin[1], origin[2])
        end_pnt = gp_Pnt(end_point[0], end_point[1], end_point[2])
        
        edge = BRepBuilderAPI_MakeEdge(start_pnt, end_pnt).Edge()
        line_ais = AIS_Shape(edge)
        
        # Create arrow head (cone)
        cone_height = length * 0.15
        cone_radius = length * 0.05
        
        # Position cone at the end of the arrow
        cone_tip = end_point
        cone_base_center = end_point - direction * cone_height
        
        # Create cone with axis pointing in force direction
        axis_dir = gp_Dir(direction[0], direction[1], direction[2])
        cone_base_pnt = gp_Pnt(cone_base_center[0], cone_base_center[1], cone_base_center[2])
        cone_axis = gp_Ax2(cone_base_pnt, axis_dir)
        
        cone = BRepPrimAPI_MakeCone(cone_axis, cone_radius, 0.0, cone_height).Shape()
        cone_ais = AIS_Shape(cone)
        
        # Set color and display properties
        self.display.Context.SetColor(line_ais, self.FORCE_COLOR, False)
        self.display.Context.SetColor(cone_ais, self.FORCE_COLOR, False)
        
        # Set line width (increased for visibility)
        self.display.Context.SetWidth(line_ais, 5.0, False)
        
        # Display
        if visible:
            self.display.Context.Display(line_ais, False)
            self.display.Context.Display(cone_ais, False)
        
        # Store references
        self.force_shapes[force.name] = [line_ais, cone_ais]
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"Force '{force.name}' rendered at {origin} with magnitude {force.magnitude}N")
    
    def remove_force(self, force_name: str):
        """
        Remove a force visualization from the viewer
        
        Args:
            force_name: Name of the force to remove
        """
        if force_name in self.force_shapes:
            shapes = self.force_shapes[force_name]
            for shape in shapes:
                if self.display.Context.IsDisplayed(shape):
                    self.display.Context.Remove(shape, False)
            
            del self.force_shapes[force_name]
            self.display.Context.UpdateCurrentViewer()
            print(f"Force '{force_name}' removed from viewer")
    
    def set_force_visibility(self, force_name: str, visible: bool):
        """
        Set visibility of a specific force
        
        Args:
            force_name: Name of the force
            visible: True to show, False to hide
        """
        if force_name in self.force_shapes:
            shapes = self.force_shapes[force_name]
            for shape in shapes:
                if visible:
                    if not self.display.Context.IsDisplayed(shape):
                        self.display.Context.Display(shape, False)
                else:
                    if self.display.Context.IsDisplayed(shape):
                        self.display.Context.Erase(shape, False)
            self.display.Context.UpdateCurrentViewer()
    
    def clear_all(self):
        """Clear all force visualizations"""
        force_names = list(self.force_shapes.keys())
        for name in force_names:
            self.remove_force(name)
        print("All forces cleared from viewer")
