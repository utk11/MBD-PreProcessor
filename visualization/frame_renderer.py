"""
Frame rendering for coordinate frames visualization
"""

from typing import Dict, Optional
import numpy as np
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Ax2
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.Graphic3d import Graphic3d_MaterialAspect, Graphic3d_NOM_PLASTIC
from core.data_structures import Frame


class FrameRenderer:
    """Handles rendering of coordinate frames as RGB axes"""
    
    def __init__(self, display):
        """
        Initialize the frame renderer
        
        Args:
            display: The OCC display object
        """
        self.display = display
        self.frame_shapes: Dict[str, list] = {}  # frame_name -> [AIS_Shape objects]
        self.frame_visible: Dict[str, bool] = {}  # frame_name -> visibility state
        self.frame_highlighted: Dict[str, bool] = {}  # frame_name -> highlight state
        self.frame_original_colors: Dict[str, list] = {}  # frame_name -> [original colors]
        self.unit_scale = 1.0  # Scale factor (Meters per Model Unit)
        
        # Default axis parameters
        self.axis_length = 0.005  # Will be scaled based on model size
        self.axis_radius = 0.0001  # Cylinder radius
        self.arrow_length = 0.00075  # Cone length
        self.arrow_radius = 0.00025  # Cone base radius
        
        print("FrameRenderer initialized")
    
    def set_axis_scale(self, scale: float):
        """
        Set the scale for axis length
        
        Args:
            scale: Scale factor (e.g., 0.1 for 10% of model size)
        """
        self.axis_length = scale
        self.axis_radius = scale * 0.02
        self.arrow_length = scale * 0.15
        self.arrow_radius = scale * 0.05
    
    def set_unit_scale(self, scale: float):
        """
        Set the unit scale (Meters per Model Unit)
        Used to convert frame positions (Meters) to Viewer coordinates (Model Units)
        """
        self.unit_scale = scale
        print(f"FrameRenderer unit scale set to {scale}")

    def render_frame(self, frame: Frame, visible: bool = True):
        """
        Render a coordinate frame as RGB axes
        
        Args:
            frame: The Frame object to render
            visible: Whether the frame should be initially visible
        """
        # Remove existing frame if already rendered
        if frame.name in self.frame_shapes:
            self.remove_frame(frame.name)
        
        shapes = []
        
        # Scale origin from Meters to Model Units for display
        # origin (meters) / scale (meters/unit) = origin (units)
        render_origin = frame.origin / self.unit_scale
        
        # X-axis (Red)
        x_axis = frame.get_x_axis()
        x_shapes = self._create_axis(render_origin, x_axis, Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB))
        shapes.extend(x_shapes)
        
        # Y-axis (Green)
        y_axis = frame.get_y_axis()
        y_shapes = self._create_axis(render_origin, y_axis, Quantity_Color(0.0, 1.0, 0.0, Quantity_TOC_RGB))
        shapes.extend(y_shapes)
        
        # Z-axis (Blue)
        z_axis = frame.get_z_axis()
        z_shapes = self._create_axis(render_origin, z_axis, Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB))
        shapes.extend(z_shapes)
        
        # Store shapes and their original colors
        self.frame_shapes[frame.name] = shapes
        self.frame_visible[frame.name] = visible
        self.frame_highlighted[frame.name] = False
        
        # Store original colors (Red, Red, Green, Green, Blue, Blue for X, Y, Z axes)
        original_colors = [
            Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB),  # X cylinder
            Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB),  # X cone
            Quantity_Color(0.0, 1.0, 0.0, Quantity_TOC_RGB),  # Y cylinder
            Quantity_Color(0.0, 1.0, 0.0, Quantity_TOC_RGB),  # Y cone
            Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB),  # Z cylinder
            Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB)   # Z cone
        ]
        self.frame_original_colors[frame.name] = original_colors
        
        # Display shapes
        if visible:
            for shape in shapes:
                self.display.Context.Display(shape, False)
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"Frame '{frame.name}' rendered at {frame.origin} (visible: {visible})")
    
    def _create_axis(self, origin: np.ndarray, direction: np.ndarray, color: Quantity_Color) -> list:
        """
        Create an axis as a cylinder with arrow head
        
        Args:
            origin: Starting point of the axis
            direction: Direction vector (normalized)
            color: Color for the axis
            
        Returns:
            List of AIS_Shape objects [cylinder, cone]
        """
        shapes = []
        
        # Normalize direction
        direction = direction / np.linalg.norm(direction)
        
        # Create cylinder for axis shaft
        cylinder_end = origin + direction * self.axis_length
        
        # OCC uses gp_Ax2 for positioning (origin + Z direction)
        cylinder_origin = gp_Pnt(origin[0], origin[1], origin[2])
        cylinder_dir = gp_Dir(direction[0], direction[1], direction[2])
        cylinder_ax = gp_Ax2(cylinder_origin, cylinder_dir)
        
        cylinder = BRepPrimAPI_MakeCylinder(cylinder_ax, self.axis_radius, self.axis_length).Shape()
        cylinder_ais = AIS_Shape(cylinder)
        cylinder_ais.SetColor(color)
        cylinder_ais.SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NOM_PLASTIC))
        shapes.append(cylinder_ais)
        
        # Create cone for arrow head
        cone_origin = gp_Pnt(cylinder_end[0], cylinder_end[1], cylinder_end[2])
        cone_ax = gp_Ax2(cone_origin, cylinder_dir)
        
        cone = BRepPrimAPI_MakeCone(cone_ax, self.arrow_radius, 0.0, self.arrow_length).Shape()
        cone_ais = AIS_Shape(cone)
        cone_ais.SetColor(color)
        cone_ais.SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NOM_PLASTIC))
        shapes.append(cone_ais)
        
        return shapes
    
    def set_frame_visibility(self, frame_name: str, visible: bool):
        """
        Toggle visibility of a frame
        
        Args:
            frame_name: Name of the frame
            visible: True to show, False to hide
        """
        if frame_name not in self.frame_shapes:
            print(f"Warning: Frame '{frame_name}' not found")
            return
        
        self.frame_visible[frame_name] = visible
        
        for shape in self.frame_shapes[frame_name]:
            if visible:
                self.display.Context.Display(shape, False)
            else:
                self.display.Context.Erase(shape, False)
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"Frame '{frame_name}' visibility: {'shown' if visible else 'hidden'}")
    
    def set_all_frames_visibility(self, visible: bool):
        """
        Toggle visibility of all frames
        
        Args:
            visible: True to show all frames, False to hide all frames
        """
        for frame_name in list(self.frame_shapes.keys()):
            self.frame_visible[frame_name] = visible
            
            for shape in self.frame_shapes[frame_name]:
                if visible:
                    self.display.Context.Display(shape, False)
                else:
                    self.display.Context.Erase(shape, False)
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"All frames visibility set to: {'shown' if visible else 'hidden'}")
    
    def highlight_frame(self, frame_name: str, highlighted: bool = True):
        """
        Toggle highlight on a frame by changing its colors to bright yellow/white
        
        Args:
            frame_name: Name of the frame
            highlighted: True to highlight, False to unhighlight
        """
        if frame_name not in self.frame_shapes:
            print(f"Warning: Frame '{frame_name}' not found for highlighting")
            return
        
        shapes = self.frame_shapes[frame_name]
        
        if highlighted:
            # Apply bright yellow highlight to all axes
            highlight_color = Quantity_Color(1.0, 1.0, 0.0, Quantity_TOC_RGB)  # Bright yellow
            for shape in shapes:
                shape.SetColor(highlight_color)
                self.display.Context.Redisplay(shape, False)
            self.frame_highlighted[frame_name] = True
            print(f"Frame '{frame_name}' highlighted")
        else:
            # Restore original colors
            if frame_name in self.frame_original_colors:
                original_colors = self.frame_original_colors[frame_name]
                for i, shape in enumerate(shapes):
                    if i < len(original_colors):
                        shape.SetColor(original_colors[i])
                        self.display.Context.Redisplay(shape, False)
            self.frame_highlighted[frame_name] = False
            print(f"Frame '{frame_name}' unhighlighted")
        
        self.display.Context.UpdateCurrentViewer()
    
    def remove_frame(self, frame_name: str):
        """
        Remove a frame from the display
        
        Args:
            frame_name: Name of the frame to remove
        """
        if frame_name not in self.frame_shapes:
            return
        
        for shape in self.frame_shapes[frame_name]:
            self.display.Context.Erase(shape, True)
        
        del self.frame_shapes[frame_name]
        del self.frame_visible[frame_name]
        if frame_name in self.frame_highlighted:
            del self.frame_highlighted[frame_name]
        if frame_name in self.frame_original_colors:
            del self.frame_original_colors[frame_name]
        
        self.display.Context.UpdateCurrentViewer()
        
        print(f"Frame '{frame_name}' removed")
    
    def clear_all_frames(self):
        """Remove all frames from the display"""
        frame_names = list(self.frame_shapes.keys())
        for frame_name in frame_names:
            self.remove_frame(frame_name)
        
        print("All frames cleared")
