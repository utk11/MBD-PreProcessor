"""
Edge renderer for highlighting selected edges in the 3D viewer
"""

from typing import Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.TopoDS import TopoDS_Edge
from OCC.Display.OCCViewer import Viewer3d


class EdgeRenderer:
    """Handles edge highlighting in the 3D viewer"""
    
    # Colors for highlighting
    HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB)  # Red
    
    def __init__(self, display):
        """
        Initialize the edge renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.current_ais: Optional[AIS_Shape] = None
        
    def highlight_edge(self, edge: TopoDS_Edge):
        """
        Highlight a specific edge
        
        Args:
            edge: TopoDS_Edge to highlight
        """
        self.clear_highlight()
        
        try:
            # Create a new AIS shape for the edge
            self.current_ais = AIS_Shape(edge)
            
            # Display it
            self.display.Context.Display(self.current_ais, False)
            
            # Set attributes
            self.display.Context.SetColor(self.current_ais, self.HIGHLIGHT_COLOR, False)
            self.display.Context.SetWidth(self.current_ais, 3.0, False)  # Thicker line
            
            # Update viewer
            self.display.Context.UpdateCurrentViewer()
            print("Edge highlighted")
            
        except Exception as e:
            print(f"Error highlighting edge: {e}")
            self.current_ais = None
            
    def clear_highlight(self):
        """Remove the current edge highlight"""
        if self.current_ais:
            self.display.Context.Erase(self.current_ais, False)
            self.display.Context.UpdateCurrentViewer()
            self.current_ais = None
