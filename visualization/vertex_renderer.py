"""
Vertex renderer for highlighting selected vertices in the 3D viewer
Increment 27: Vertex Selection Mode
"""

from typing import Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.TopoDS import TopoDS_Vertex
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Pnt
from OCC.Display.OCCViewer import Viewer3d


class VertexRenderer:
    """Handles vertex highlighting in the 3D viewer"""
    
    # Color for highlighting - Cyan to distinguish from red COM marker
    HIGHLIGHT_COLOR = Quantity_Color(0.0, 0.8, 1.0, Quantity_TOC_RGB)  # Cyan
    
    def __init__(self, display):
        """
        Initialize the vertex renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.current_ais: Optional[AIS_Shape] = None
        self.unit_scale = 1.0
        self.highlight_size = 0.005 # Default 5mm if scale is 1
        
    def set_unit_scale(self, scale: float):
        """Set the unit scale to adjust highlight size properly"""
        self.unit_scale = scale
        # Set highlight size to roughly 5mm (0.005m) converted to model units
        self.highlight_size = 0.001 / scale
        
    def highlight_vertex(self, vertex: TopoDS_Vertex):
        """
        Highlight a specific vertex by rendering a small sphere at its location
        
        Args:
            vertex: TopoDS_Vertex to highlight
        """
        self.clear_highlight()
        
        try:
            # Get vertex coordinates
            pnt = BRep_Tool.Pnt(vertex)
            
            # Create a small sphere at the vertex location
            # Use dynamically set size
            sphere = BRepPrimAPI_MakeSphere(pnt, self.highlight_size).Shape()
            
            # Create AIS shape for the sphere
            self.current_ais = AIS_Shape(sphere)
            
            # Display it
            self.display.Context.Display(self.current_ais, False)
            
            # Set color
            self.display.Context.SetColor(self.current_ais, self.HIGHLIGHT_COLOR, False)
            
            # Update viewer
            self.display.Context.UpdateCurrentViewer()
            print(f"Vertex highlighted at ({pnt.X():.6f}, {pnt.Y():.6f}, {pnt.Z():.6f})")
            
        except Exception as e:
            print(f"Error highlighting vertex: {e}")
            self.current_ais = None
            
    def clear_highlight(self):
        """Remove the current vertex highlight"""
        if self.current_ais:
            self.display.Context.Erase(self.current_ais, False)
            self.display.Context.UpdateCurrentViewer()
            self.current_ais = None
