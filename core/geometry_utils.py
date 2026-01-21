"""
Geometry utility functions for face, edge, and vertex selection and analysis
Increment 14: Face Selection Mode
Increment 27: Vertex Selection Mode
"""

from typing import Optional, Tuple, List
import numpy as np
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Face, TopoDS_Edge, TopoDS_Vertex
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.BRepLProp import BRepLProp_CLProps
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.gp import gp_Pnt
from OCC.Core.BRep import BRep_Tool
from core.data_structures import Frame


class FaceProperties:
    """Container for face geometric properties"""
    
    def __init__(self, face: TopoDS_Face, face_index: int, unit_scale: float = 1.0):
        """
        Initialize face properties
        
        Args:
            face: TopoDS_Face object
            face_index: Index of the face in the body
            unit_scale: Scale factor to convert model units to meters
        """
        self.face = face
        self.face_index = face_index
        self.unit_scale = unit_scale
        self.area = 0.0  # Surface area
        self.center = np.array([0.0, 0.0, 0.0])  # Center point [x, y, z]
        self.normal = np.array([0.0, 0.0, 1.0])  # Normal vector [nx, ny, nz]
        self._calculate_properties()
    
    def _calculate_properties(self):
        """Calculate area, center, and normal for the face"""
        # Calculate area using GProp
        system = GProp_GProps()
        brepgprop.SurfaceProperties(self.face, system)
        self.area = system.Mass() * (self.unit_scale ** 2)
        
        # Get center of mass
        mass_center = system.CentreOfMass()
        self.center = np.array([mass_center.X(), mass_center.Y(), mass_center.Z()]) * self.unit_scale
        
        # Calculate normal at the center point
        self._calculate_normal()
    
    def _calculate_normal(self):
        """Calculate the normal vector at the face center"""
        try:
            from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
            from OCC.Core.BRepLProp import BRepLProp_SLProps
            from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
            
            # Get surface from face
            surface_adaptor = BRepAdaptor_Surface(self.face)
            
            # Get parametric bounds
            u_min = surface_adaptor.FirstUParameter()
            u_max = surface_adaptor.LastUParameter()
            v_min = surface_adaptor.FirstVParameter()
            v_max = surface_adaptor.LastVParameter()
            
            # Use middle of parametric space
            u_mid = (u_min + u_max) / 2.0
            v_mid = (v_min + v_max) / 2.0
            
            # Evaluate properties at the midpoint
            # Degree 2 is often better for checking curvature, but 1 is sufficient for normal
            props = BRepLProp_SLProps(surface_adaptor, 1, 1e-7)
            props.SetParameters(u_mid, v_mid)
            
            # Get normal if defined
            if props.IsNormalDefined():
                n_dir = props.Normal()
                self.normal = np.array([n_dir.X(), n_dir.Y(), n_dir.Z()])
            else:
                # Use default normal if not defined (e.g. singularity)
                print(f"Warning: Normal not defined for face {self.face_index} at ({u_mid}, {v_mid}).")
                self.normal = np.array([0.0, 0.0, 1.0])
                
        except Exception as e:
            # Default normal if anything fails
            print(f"Warning: Normal calculation failed for face {self.face_index}: {e}. Using default normal (0,0,1).")
            self.normal = np.array([0.0, 0.0, 1.0])
    
    def __repr__(self):
        return (f"FaceProperties(index={self.face_index}, "
                f"area={self.area:.6e}, "
                f"center={self.center}, "
                f"normal={self.normal})")


class VertexProperties:
    """Container for vertex geometric properties"""
    
    def __init__(self, vertex: TopoDS_Vertex, vertex_index: int, unit_scale: float = 1.0):
        """
        Initialize vertex properties
        
        Args:
            vertex: TopoDS_Vertex object
            vertex_index: Index of the vertex in the body
            unit_scale: Scale factor to convert model units to meters
        """
        self.vertex = vertex
        self.vertex_index = vertex_index
        self.unit_scale = unit_scale
        self.coordinates = np.array([0.0, 0.0, 0.0])  # Vertex position [x, y, z] in meters
        self._calculate_properties()
    
    def _calculate_properties(self):
        """Calculate coordinates for the vertex"""
        try:
            # Get point from vertex using BRep_Tool
            pnt = BRep_Tool.Pnt(self.vertex)
            # Scale to meters
            self.coordinates = np.array([pnt.X() * self.unit_scale, 
                                        pnt.Y() * self.unit_scale, 
                                        pnt.Z() * self.unit_scale])
        except Exception as e:
            print(f"Warning: Could not calculate vertex properties: {e}")
            self.coordinates = np.array([0.0, 0.0, 0.0])
    
    def __repr__(self):
        return (f"VertexProperties(index={self.vertex_index}, "
                f"coordinates={self.coordinates})")


class EdgeProperties:
    """Container for edge geometric properties"""
    
    def __init__(self, edge: TopoDS_Edge, edge_index: int, unit_scale: float = 1.0):
        """
        Initialize edge properties
        
        Args:
            edge: TopoDS_Edge object
            edge_index: Index of the edge in the body
            unit_scale: Scale factor to convert model units to meters
        """
        self.edge = edge
        self.edge_index = edge_index
        self.unit_scale = unit_scale
        self.length = 0.0  # Edge length
        self.midpoint = np.array([0.0, 0.0, 0.0])  # Midpoint [x, y, z]
        self.direction = np.array([1.0, 0.0, 0.0])  # Direction vector
        self._calculate_properties()
    
    def _calculate_properties(self):
        """Calculate length, midpoint, and direction for the edge"""
        from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
        from OCC.Core.BRepLProp import BRepLProp_CLProps
        from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnCurve
        
        try:
            # Get edge curve
            from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
            curve_adaptor = BRepAdaptor_Curve(self.edge)
            curve = curve_adaptor.Curve().Curve()
            
            # Get parameter range
            u_min = curve_adaptor.FirstParameter()
            u_max = curve_adaptor.LastParameter()
            
            # Calculate length as distance between endpoints
            start_pnt = curve.Value(u_min)
            end_pnt = curve.Value(u_max)
            
            start = np.array([start_pnt.X(), start_pnt.Y(), start_pnt.Z()])
            end = np.array([end_pnt.X(), end_pnt.Y(), end_pnt.Z()])
            
            self.length = np.linalg.norm(end - start) * self.unit_scale
            
            # Midpoint is average of start and end
            self.midpoint = ((start + end) / 2.0) * self.unit_scale
            
            # Direction is normalized vector from start to end
            direction = end - start
            length = np.linalg.norm(direction)
            if length > 1e-10:
                self.direction = direction / length
            else:
                self.direction = np.array([1.0, 0.0, 0.0])
                
        except Exception as e:
            print(f"Warning: Could not calculate edge properties: {e}")
            self.direction = np.array([1.0, 0.0, 0.0])
    
    def __repr__(self):
        return (f"EdgeProperties(index={self.edge_index}, "
                f"length={self.length:.6f}, "
                f"midpoint={self.midpoint}, "
                f"direction={self.direction})")


class GeometryUtils:
    """Utility functions for geometry analysis"""
    
    @staticmethod
    def extract_faces(shape: TopoDS_Shape, unit_scale: float = 1.0) -> List[FaceProperties]:
        """
        Extract all faces from a shape
        
        Args:
            shape: TopoDS_Shape object
            unit_scale: Scale factor to convert model units to meters
            
        Returns:
            List of FaceProperties objects
        """
        from OCC.Core.TopoDS import topods
        
        faces = []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        face_index = 0
        
        while explorer.More():
            face = topods.Face(explorer.Current())
            props = FaceProperties(face, face_index, unit_scale)
            faces.append(props)
            face_index += 1
            explorer.Next()
        
        return faces
    
    @staticmethod
    def extract_edges(shape: TopoDS_Shape, unit_scale: float = 1.0) -> List[EdgeProperties]:
        """
        Extract all edges from a shape
        
        Args:
            shape: TopoDS_Shape object
            unit_scale: Scale factor to convert model units to meters
            
        Returns:
            List of EdgeProperties objects
        """
        from OCC.Core.TopoDS import topods
        
        edges = []
        explorer = TopExp_Explorer(shape, TopAbs_EDGE)
        edge_index = 0
        
        while explorer.More():
            edge = topods.Edge(explorer.Current())
            props = EdgeProperties(edge, edge_index, unit_scale)
            edges.append(props)
            edge_index += 1
            explorer.Next()
        
        return edges
    
    @staticmethod
    def extract_vertices(shape: TopoDS_Shape, unit_scale: float = 1.0) -> List[VertexProperties]:
        """
        Extract all vertices from a shape
        
        Args:
            shape: TopoDS_Shape object
            unit_scale: Scale factor to convert model units to meters
            
        Returns:
            List of VertexProperties objects
        """
        from OCC.Core.TopoDS import topods
        
        vertices = []
        explorer = TopExp_Explorer(shape, TopAbs_VERTEX)
        vertex_index = 0
        
        while explorer.More():
            vertex = topods.Vertex(explorer.Current())
            props = VertexProperties(vertex, vertex_index, unit_scale)
            vertices.append(props)
            vertex_index += 1
            explorer.Next()
        
        return vertices
    
    @staticmethod
    def get_face_by_index(shape: TopoDS_Shape, face_index: int) -> Optional[TopoDS_Face]:
        """
        Get a specific face by index
        
        Args:
            shape: TopoDS_Shape object
            face_index: Index of the face to retrieve
            
        Returns:
            TopoDS_Face if found, None otherwise
        """
        from OCC.Core.TopoDS import topods
        
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        current_index = 0
        
        while explorer.More():
            if current_index == face_index:
                return topods.Face(explorer.Current())
            current_index += 1
            explorer.Next()
        
        return None
    
    @staticmethod
    def get_edge_by_index(shape: TopoDS_Shape, edge_index: int) -> Optional[TopoDS_Edge]:
        """
        Get a specific edge by index
        
        Args:
            shape: TopoDS_Shape object
            edge_index: Index of the edge to retrieve
            
        Returns:
            TopoDS_Edge if found, None otherwise
        """
        from OCC.Core.TopoDS import topods
        
        explorer = TopExp_Explorer(shape, TopAbs_EDGE)
        current_index = 0
        
        while explorer.More():
            if current_index == edge_index:
                return topods.Edge(explorer.Current())
            current_index += 1
            explorer.Next()
        
        return None
    
    @staticmethod
    def get_vertex_by_index(shape: TopoDS_Shape, vertex_index: int) -> Optional[TopoDS_Vertex]:
        """
        Get a specific vertex by index
        
        Args:
            shape: TopoDS_Shape object
            vertex_index: Index of the vertex to retrieve
            
        Returns:
            TopoDS_Vertex if found, None otherwise
        """
        from OCC.Core.TopoDS import topods
        
        explorer = TopExp_Explorer(shape, TopAbs_VERTEX)
        current_index = 0
        
        while explorer.More():
            if current_index == vertex_index:
                return topods.Vertex(explorer.Current())
            current_index += 1
            explorer.Next()
        
        return None

    @staticmethod
    def frame_from_face(face_props: FaceProperties, name: str = "Face Frame", unit_scale: float = 1.0) -> Frame:
        """
        Create a Frame whose origin is the face center and Z-axis is the face normal.
        X-axis is aligned as closely as possible to the Global X-axis (Projection strategy).
        
        Args:
            face_props: FaceProperties object
            name: Name for the frame
            unit_scale: Scale factor to convert model units to meters (deprecated, used in properties)
        """
        # Z-axis is the face normal
        z_axis = np.array(face_props.normal, dtype=float)
        if np.linalg.norm(z_axis) < 1e-8:
            z_axis = np.array([0.0, 0.0, 1.0])
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Strategy: Align Frame X with Global X as much as possible
        # Project Global X (1,0,0) onto the plane perpendicular to Normal
        target_x = np.array([1.0, 0.0, 0.0])
        
        # If normal is too parallel to Global X (dot product ~ 1), switch target to Global Y
        if abs(np.dot(target_x, z_axis)) > 0.99:
            target_x = np.array([0.0, 1.0, 0.0])

        # Project target vector onto the plane: v_proj = v - (v . n) * n
        x_axis = target_x - np.dot(target_x, z_axis) * z_axis
        
        # Normalize X-axis
        if np.linalg.norm(x_axis) < 1e-8:
            # Fallback (should ideally not happen with the check above)
            x_axis = np.array([1.0, 0.0, 0.0]) 
            if abs(np.dot(x_axis, z_axis)) > 0.9:
                 x_axis = np.array([0.0, 1.0, 0.0])
            x_axis = np.cross(z_axis, x_axis) # Force orthogonality

        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Y-axis is Z cross X (Right-hand rule)
        y_axis = np.cross(z_axis, x_axis)

        rotation = np.column_stack((x_axis, y_axis, z_axis))
        origin = np.array(face_props.center, dtype=float)  # Already scaled to meters in FaceProperties

        return Frame(origin=origin, rotation_matrix=rotation, name=name)

    @staticmethod
    def frame_from_edge(edge_props: EdgeProperties, name: str = "Edge Frame", unit_scale: float = 1.0) -> Frame:
        """
        Create a Frame whose origin is the edge midpoint and Z-axis is the edge direction.
        X-axis is aligned as closely as possible to the Global X-axis (Projection strategy).
        
        Args:
            edge_props: EdgeProperties object
            name: Name for the frame
            unit_scale: Scale factor to convert model units to meters (deprecated, used in properties)
        """
        # Z-axis is the edge direction
        z_axis = np.array(edge_props.direction, dtype=float)
        if np.linalg.norm(z_axis) < 1e-8:
            z_axis = np.array([0.0, 0.0, 1.0])
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Strategy: Align Frame X with Global X as much as possible
        target_x = np.array([1.0, 0.0, 0.0])
        
        # If direction is too parallel to Global X, switch target to Global Y
        if abs(np.dot(target_x, z_axis)) > 0.99:
            target_x = np.array([0.0, 1.0, 0.0])

        # Project target vector onto the plane: v_proj = v - (v . n) * n
        x_axis = target_x - np.dot(target_x, z_axis) * z_axis

        # Normalize X-axis
        if np.linalg.norm(x_axis) < 1e-8:
             # Fallback
            x_axis = np.array([1.0, 0.0, 0.0]) 
            if abs(np.dot(x_axis, z_axis)) > 0.9:
                 x_axis = np.array([0.0, 1.0, 0.0])
            x_axis = np.cross(z_axis, x_axis)

        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Y-axis is Z cross X
        y_axis = np.cross(z_axis, x_axis)

        rotation = np.column_stack((x_axis, y_axis, z_axis))
        origin = np.array(edge_props.midpoint, dtype=float)  # Already scaled to meters in EdgeProperties

        return Frame(origin=origin, rotation_matrix=rotation, name=name)
    
    @staticmethod
    def frame_from_vertex(vertex_props: VertexProperties, name: str = "Vertex Frame") -> Frame:
        """
        Create a Frame at the vertex location with identity rotation (aligned with world axes).
        Since vertices don't have inherent orientation, the frame is aligned with the world frame.
        
        Args:
            vertex_props: VertexProperties object
            name: Name for the frame
            
        Returns:
            Frame object at vertex location with identity rotation
        """
        origin = np.array(vertex_props.coordinates, dtype=float)
        rotation = np.eye(3)  # Identity rotation - aligned with world frame
        
        return Frame(origin=origin, rotation_matrix=rotation, name=name)
