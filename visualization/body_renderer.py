"""
Body renderer for highlighting and displaying bodies in the 3D viewer
"""

from typing import Optional, Dict
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Pnt
from OCC.Display.OCCViewer import Viewer3d
from core.data_structures import RigidBody


class BodyRenderer:
    """Handles body rendering and highlighting in the 3D viewer"""
    
    # Colors for highlighting
    NORMAL_COLOR = Quantity_Color(0.8, 0.8, 0.8, Quantity_TOC_RGB)  # Light gray
    HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.9, 0.0, Quantity_TOC_RGB)  # Yellow
    COM_COLOR = Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB)  # Red for COM marker
    
    def __init__(self, display):
        """
        Initialize the body renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.body_ais_shapes: Dict[int, AIS_Shape] = {}  # Map body_id -> AIS_Shape
        self.currently_highlighted_id: Optional[int] = None
        self.com_marker_ais: Optional[AIS_Shape] = None  # COM sphere marker
        self.com_marker_visible: bool = True  # COM visibility state
        self.bodies_dict: Dict[int, RigidBody] = {}  # Map body_id -> RigidBody
        self.unit_scale = 1.0  # Scale factor (Meters per Model Unit)

    def set_unit_scale(self, scale: float):
        """
        Set the unit scale (Meters per Model Unit)
        """
        self.unit_scale = scale
    
    def display_bodies(self, bodies: list[RigidBody]):
        """
        Display all bodies in the viewer
        
        Args:
            bodies: List of RigidBody objects to display
        """
        # Clear previous AIS shapes
        self.body_ais_shapes.clear()
        self.bodies_dict.clear()
        self.currently_highlighted_id = None
        self._clear_com_marker()
        
        # Display each body and store its AIS_Shape
        for body in bodies:
            if body.shape is None:
                continue

            ais_shape = AIS_Shape(body.shape)
            
            # Display the shape
            self.display.Context.Display(ais_shape, False)
            
            # CRITICAL: Activate selection mode 0 (entire shape) to make it selectable
            # Mode 0 = select the whole shape/body
            self.display.Context.Activate(ais_shape, 0, False)
            
            # Set default color
            self.display.Context.SetColor(ais_shape, self.NORMAL_COLOR, False)
            
            print(f"Body {body.id} ({body.name}): Displayed and activated for selection, AIS={ais_shape}")
            
            # Store mapping
            self.body_ais_shapes[body.id] = ais_shape
            self.bodies_dict[body.id] = body
        
        # Update display
        self.display.Context.UpdateCurrentViewer()
        print(f"All {len(bodies)} bodies displayed and activated for selection")
        
    def highlight_body(self, body_id: int):
        """
        Highlight a specific body by changing its color
        
        Args:
            body_id: ID of the body to highlight
        """
        # Unhighlight the previously highlighted body
        if self.currently_highlighted_id is not None:
            self._unhighlight_body(self.currently_highlighted_id)
        
        # Highlight the new body
        if body_id in self.body_ais_shapes:
            ais_shape = self.body_ais_shapes[body_id]
            self.display.Context.SetColor(ais_shape, self.HIGHLIGHT_COLOR, True)
            self.currently_highlighted_id = body_id
            print(f"Body {body_id} highlighted in viewer")
            
            # Show COM marker for the selected body
            self._update_com_marker(body_id)
        else:
            print(f"Warning: Body {body_id} not found in renderer")
            
    def _unhighlight_body(self, body_id: int):
        """
        Remove highlighting from a body by restoring its normal color
        
        Args:
            body_id: ID of the body to unhighlight
        """
        if body_id in self.body_ais_shapes:
            ais_shape = self.body_ais_shapes[body_id]
            self.display.Context.SetColor(ais_shape, self.NORMAL_COLOR, True)
            
    def set_body_visibility(self, body_id: int, visible: bool):
        """
        Set visibility of a specific body
        
        Args:
            body_id: ID of the body
            visible: True to show, False to hide
        """
        if body_id in self.body_ais_shapes:
            ais_shape = self.body_ais_shapes[body_id]
            if visible:
                if not self.display.Context.IsDisplayed(ais_shape):
                    self.display.Context.Display(ais_shape, True)
            else:
                if self.display.Context.IsDisplayed(ais_shape):
                    self.display.Context.Erase(ais_shape, True)
            print(f"Body {body_id} visibility set to {visible}")

    def clear_highlight(self):
        """Clear all highlighting"""
        if self.currently_highlighted_id is not None:
            self._unhighlight_body(self.currently_highlighted_id)
            self.currently_highlighted_id = None
        self._clear_com_marker()
            
    def clear_all(self):
        """Clear all rendered bodies"""
        self.body_ais_shapes.clear()
        self.bodies_dict.clear()
        self.currently_highlighted_id = None
        self._clear_com_marker()
        self.display.EraseAll()
    
    def set_com_visibility(self, visible: bool):
        """
        Set the visibility of the COM marker
        
        Args:
            visible: True to show COM marker, False to hide it
        """
        self.com_marker_visible = visible
        
        if visible and self.currently_highlighted_id is not None:
            # Re-create COM marker for currently selected body
            self._update_com_marker(self.currently_highlighted_id)
        else:
            # Hide COM marker
            self._clear_com_marker()
    
    def _update_com_marker(self, body_id: int):
        """
        Update the COM marker for the specified body
        
        Args:
            body_id: ID of the body to show COM for
        """
        # Clear existing COM marker
        self._clear_com_marker()
        
        # Only show if visibility is enabled
        if not self.com_marker_visible:
            return
        
        # Get the body
        body = self.bodies_dict.get(body_id)
        if body is None or body.center_of_mass is None:
            return
        
        # Get COM coordinates (Meters)
        com = body.center_of_mass
        
        # Scale to Model Units for display
        cx = com[0] / self.unit_scale
        cy = com[1] / self.unit_scale
        cz = com[2] / self.unit_scale
        
        # Calculate sphere radius based on body size
        # Use a small fixed radius or scale based on bounding box
        # Scale radius to model units too?
        sphere_radius = 0.005 / self.unit_scale  # 5mm equivalent in model units
        
        # Create sphere at COM location
        com_point = gp_Pnt(cx, cy, cz)
        sphere_shape = BRepPrimAPI_MakeSphere(com_point, sphere_radius).Shape()
        
        # Create AIS shape for the sphere
        self.com_marker_ais = AIS_Shape(sphere_shape)
        
        # Set red color for COM marker
        self.display.Context.Display(self.com_marker_ais, False)
        self.display.Context.SetColor(self.com_marker_ais, self.COM_COLOR, False)
        self.display.Context.UpdateCurrentViewer()
        
        print(f"COM marker displayed at [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}] m")
    
    def _clear_com_marker(self):
        """Remove the COM marker from the display"""
        if self.com_marker_ais is not None:
            self.display.Context.Erase(self.com_marker_ais, False)
            self.display.Context.UpdateCurrentViewer()
            self.com_marker_ais = None
    
    def remove_body(self, body_id: int):
        """
        Remove a body from the display
        
        Args:
            body_id: ID of the body to remove
        """
        if body_id in self.body_ais_shapes:
            ais_shape = self.body_ais_shapes[body_id]
            
            # Remove from display
            self.display.Context.Erase(ais_shape, False)
            self.display.Context.Remove(ais_shape, False)
            
            # Remove from mappings
            del self.body_ais_shapes[body_id]
            if body_id in self.bodies_dict:
                del self.bodies_dict[body_id]
            
            # Clear highlight if this was the highlighted body
            if self.currently_highlighted_id == body_id:
                self.currently_highlighted_id = None
                self._clear_com_marker()
            
            # Update display
            self.display.Context.UpdateCurrentViewer()
            print(f"Body {body_id} removed from renderer")
        else:
            print(f"Body {body_id} not found in renderer")
