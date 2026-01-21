
import sys
import os
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.gp import gp_Pnt
from core.geometry_utils import GeometryUtils, EdgeProperties

def test_frame_from_edge():
    print("Testing frame_from_edge...")
    
    # Create an edge along X axis: (0,0,0) to (1,0,0)
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(1, 0, 0)
    edge_shape = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    
    # Extract properties
    props = EdgeProperties(edge_shape, 0)
    print(f"Edge props: {props}")
    
    # Create frame
    frame = GeometryUtils.frame_from_edge(props, "TestEdgeFrame")
    
    print(f"Frame Origin: {frame.origin}")
    print(f"Frame Z-axis: {frame.get_z_axis()}")
    print(f"Frame X-axis: {frame.get_x_axis()}")
    
    # Checks
    # Origin should be midpoint (0.5, 0, 0)
    if not np.allclose(frame.origin, [0.5, 0, 0]):
        print("FAIL: Origin incorrect")
        return False
        
    # Z-axis should be direction (1, 0, 0)
    if not np.allclose(frame.get_z_axis(), [1, 0, 0]):
        print("FAIL: Z-axis incorrect")
        return False
        
    print("PASS: frame_from_edge calculation correct")
    return True

if __name__ == "__main__":
    if test_frame_from_edge():
        sys.exit(0)
    else:
        sys.exit(1)
