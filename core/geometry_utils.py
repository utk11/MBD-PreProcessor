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
        """Calculate area, center, and normal for the face.

        Center is the surface area centroid (CentreOfMass of the face). That
        point is allowed to sit in empty space — e.g. on a cylinder/hole axis —
        which is what joint frames need. It is NOT projected onto the skin.

        OCC returns model units; ``center`` is stored in meters.
        ``center_model`` keeps the raw OCC coordinates for display alignment.

        Normal is evaluated at the face UV midpoint (with orientation), since a
        surface normal is only defined on the surface itself.
        """
        from OCC.Core.BRepLProp import BRepLProp_SLProps
        from OCC.Core.BRepAdaptor import BRepAdaptor_Surface

        system = GProp_GProps()
        brepgprop.SurfaceProperties(self.face, system)
        self.area = system.Mass() * (self.unit_scale ** 2)

        self.center_model = np.array([0.0, 0.0, 0.0], dtype=float)
        self.center = np.array([0.0, 0.0, 0.0], dtype=float)
        self.normal = np.array([0.0, 0.0, 1.0], dtype=float)

        try:
            mass_center = system.CentreOfMass()
            self.center_model = np.array(
                [mass_center.X(), mass_center.Y(), mass_center.Z()], dtype=float
            )
            self.center = self.center_model * float(self.unit_scale)
        except Exception as e:
            print(f"Warning: face center failed for face {self.face_index}: {e}")
            self.center_model = np.zeros(3)
            self.center = np.zeros(3)

        try:
            # Restriction=True → UV domain of the face; locations applied on Value()
            surface_adaptor = BRepAdaptor_Surface(self.face, True)
            u_mid = 0.5 * (
                surface_adaptor.FirstUParameter() + surface_adaptor.LastUParameter()
            )
            v_mid = 0.5 * (
                surface_adaptor.FirstVParameter() + surface_adaptor.LastVParameter()
            )

            props = BRepLProp_SLProps(surface_adaptor, 1, 1e-7)
            props.SetParameters(float(u_mid), float(v_mid))
            if props.IsNormalDefined():
                n_dir = props.Normal()
                self.normal = np.array(
                    [n_dir.X(), n_dir.Y(), n_dir.Z()], dtype=float
                )
                try:
                    from OCC.Core.TopAbs import TopAbs_REVERSED
                    if self.face.Orientation() == TopAbs_REVERSED:
                        self.normal = -self.normal
                except Exception:
                    pass
                nrm = float(np.linalg.norm(self.normal))
                if nrm > 1e-12:
                    self.normal = self.normal / nrm
            else:
                self.normal = np.array([0.0, 0.0, 1.0], dtype=float)
        except Exception as e:
            print(f"Warning: face normal failed for face {self.face_index}: {e}")
            self.normal = np.array([0.0, 0.0, 1.0], dtype=float)
    
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
        """Calculate coordinates for the vertex.

        BRep_Tool.Pnt already returns the point with the vertex location applied.
        """
        try:
            from OCC.Core.BRep import BRep_Tool
            pnt = BRep_Tool.Pnt(self.vertex)
            self.coordinates_model = np.array(
                [pnt.X(), pnt.Y(), pnt.Z()], dtype=float
            )
            self.coordinates = self.coordinates_model * float(self.unit_scale)
        except Exception as e:
            print(f"Warning: Could not calculate vertex properties: {e}")
            self.coordinates_model = np.array([0.0, 0.0, 0.0])
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
        """Calculate length, midpoint, and direction for the edge.

        Midpoint is the natural feature center and may sit in empty space:
        - circle / ellipse → geometric center (hole/pin axis point in the edge plane)
        - other curves → length-weighted centroid (LinearProperties COM)
        - straight line → segment midpoint

        It is NOT forced onto the curve (parametric mid sits on the metal and
        looks offset for arcs and full circles).

        Direction:
        - circle / ellipse → plane normal (circle axis) — useful for revolute frames
        - line → line direction
        - else → chord or tangent fallback

        BRepAdaptor_Curve applies the edge TopLoc_Location.
        """
        try:
            from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
            from OCC.Core.GeomAbs import (
                GeomAbs_Circle,
                GeomAbs_Ellipse,
                GeomAbs_Line,
            )
            from OCC.Core.BRepLProp import BRepLProp_CLProps

            curve_adaptor = BRepAdaptor_Curve(self.edge)
            u_min = curve_adaptor.FirstParameter()
            u_max = curve_adaptor.LastParameter()
            u_mid = 0.5 * (u_min + u_max)
            ctype = curve_adaptor.GetType()

            # True curve length (not chord)
            system = GProp_GProps()
            brepgprop.LinearProperties(self.edge, system)
            self.length = float(system.Mass()) * float(self.unit_scale)

            mid = None
            direction = None

            if ctype == GeomAbs_Circle:
                circ = curve_adaptor.Circle()
                loc = circ.Location()
                mid = np.array([loc.X(), loc.Y(), loc.Z()], dtype=float)
                ax = circ.Axis().Direction()
                direction = np.array([ax.X(), ax.Y(), ax.Z()], dtype=float)
            elif ctype == GeomAbs_Ellipse:
                ell = curve_adaptor.Ellipse()
                loc = ell.Location()
                mid = np.array([loc.X(), loc.Y(), loc.Z()], dtype=float)
                ax = ell.Axis().Direction()
                direction = np.array([ax.X(), ax.Y(), ax.Z()], dtype=float)
            else:
                # Length centroid — on the segment for lines; may be off-curve
                # for bent wires (allowed; same idea as face area COM).
                mc = system.CentreOfMass()
                mid = np.array([mc.X(), mc.Y(), mc.Z()], dtype=float)

                if ctype == GeomAbs_Line:
                    line = curve_adaptor.Line()
                    d = line.Direction()
                    direction = np.array([d.X(), d.Y(), d.Z()], dtype=float)
                else:
                    start_pnt = curve_adaptor.Value(u_min)
                    end_pnt = curve_adaptor.Value(u_max)
                    start = np.array(
                        [start_pnt.X(), start_pnt.Y(), start_pnt.Z()], dtype=float
                    )
                    end = np.array(
                        [end_pnt.X(), end_pnt.Y(), end_pnt.Z()], dtype=float
                    )
                    chord = end - start
                    clen = float(np.linalg.norm(chord))
                    if clen > 1e-10:
                        direction = chord / clen

            if direction is None or float(np.linalg.norm(direction)) < 1e-12:
                try:
                    props = BRepLProp_CLProps(curve_adaptor, 1, 1e-6)
                    props.SetParameter(u_mid)
                    if props.IsTangentDefined():
                        t = props.Tangent()
                        direction = np.array([t.X(), t.Y(), t.Z()], dtype=float)
                except Exception:
                    direction = None

            if direction is None or float(np.linalg.norm(direction)) < 1e-12:
                direction = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                direction = np.asarray(direction, dtype=float)
                direction = direction / float(np.linalg.norm(direction))

            # Match edge orientation when OCC marks it reversed
            try:
                from OCC.Core.TopAbs import TopAbs_REVERSED
                if self.edge.Orientation() == TopAbs_REVERSED:
                    direction = -direction
            except Exception:
                pass

            self.midpoint_model = np.asarray(mid, dtype=float)
            self.midpoint = self.midpoint_model * float(self.unit_scale)
            self.direction = direction

        except Exception as e:
            print(f"Warning: Could not calculate edge properties: {e}")
            self.midpoint_model = np.array([0.0, 0.0, 0.0])
            self.midpoint = np.array([0.0, 0.0, 0.0])
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
        Create a Frame at the edge feature center (may be empty space for circles)
        with Z along edge_props.direction (circle axis for circular edges, tangent/line dir otherwise).
        X-axis is aligned as closely as possible to the Global X-axis (Projection strategy).
        
        Args:
            edge_props: EdgeProperties object
            name: Name for the frame
            unit_scale: Scale factor to convert model units to meters (deprecated, used in properties)
        """
        # Z-axis is the edge direction (or circle plane normal for circles)
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
