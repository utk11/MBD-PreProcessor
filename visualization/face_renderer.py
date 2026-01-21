"""
Face renderer for highlighting selected faces in the 3D viewer
"""

from typing import Optional
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.TopoDS import TopoDS_Face
from OCC.Display.OCCViewer import Viewer3d


class FaceRenderer:
    """Handles face highlighting in the 3D viewer"""
    
    # Colors for highlighting
    HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.9, 0.0, Quantity_TOC_RGB)  # Yellow
    
    def __init__(self, display):
        """
        Initialize the face renderer
        
        Args:
            display: The OCC display object from qtViewer3d
        """
        self.display = display
        self.current_ais: Optional[AIS_Shape] = None
        
    def highlight_face(self, face: TopoDS_Face):
        """
        Highlight a specific face
        
        Args:
            face: TopoDS_Face to highlight
        """
        self.clear_highlight()
        
        try:
            # Create a new AIS shape for the face
            self.current_ais = AIS_Shape(face)
            
            # Display it
            self.display.Context.Display(self.current_ais, False)
            
            # Set attributes
            self.display.Context.SetColor(self.current_ais, self.HIGHLIGHT_COLOR, False)
            
            # Optional: Set transparency to allow seeing texture/geometry behind if needed
            # self.display.Context.SetTransparency(self.current_ais, 0.2, False)
            
            # Update viewer
            self.display.Context.UpdateCurrentViewer()
            print("Face highlighted")
            
        except Exception as e:
            print(f"Error highlighting face: {e}")
            self.current_ais = None
            
    def clear_highlight(self):
        """Remove the current face highlight"""
        if self.current_ais:
            self.display.Context.Erase(self.current_ais, False)
            self.display.Context.UpdateCurrentViewer()
            self.current_ais = None
