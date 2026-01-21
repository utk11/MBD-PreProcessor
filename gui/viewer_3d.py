"""
Enhanced 3D Viewer with Mouse Click Selection
Increment 13: Mouse Click Selection in Viewer
Increment 14: Face and Edge Selection
Increment 27: Vertex Selection
"""

from typing import Optional, Callable, Dict, Tuple
from PySide6.QtCore import Signal, QObject
from OCC.Display.qtDisplay import qtViewer3d
from OCC.Core.AIS import AIS_Shape
from OCC.Core.Aspect import Aspect_GT_Rectangular, Aspect_TOTP_LEFT_LOWER
from OCC.Core.Quantity import Quantity_Color, Quantity_NOC_BLACK
from OCC.Core.V3d import V3d_ZBUFFER
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.TopoDS import TopoDS_Face, TopoDS_Edge, TopoDS_Vertex
from core.data_structures import RigidBody


class SelectableViewer3d(qtViewer3d):
    """
    Enhanced 3D viewer with mouse click selection capabilities for bodies, faces, and edges
    """

    def __init__(self, parent=None):
        """
        Initialize the selectable 3D viewer

        Args:
            parent: Parent widget
        """
        super().__init__(parent)
        # Improve pick hit area to make selection less fussy on high-DPI screens
        # Reduced from 12 to 4 to prevent selecting nearby faces instead of the one under cursor
        self._display.Context.SetPixelTolerance(4)

        # Display the orientation triedron (X, Y, Z axis frame) in the bottom-left corner
        # Default scale is 0.1. User requested significantly bigger. Setting to 0.30.
        self._display.View.TriedronDisplay(
            Aspect_TOTP_LEFT_LOWER,
            Quantity_Color(Quantity_NOC_BLACK),
            0.50,
            V3d_ZBUFFER
        )

        # Store body mapping for click selection
        self.body_ais_map: Dict[int, AIS_Shape] = {}  # body_id -> AIS_Shape
        self.ais_body_map: Dict[AIS_Shape, int] = {}  # AIS_Shape -> body_id

        # Store body shape references for face/edge selection
        self.bodies_dict: Dict[int, RigidBody] = {}  # body_id -> RigidBody

        # Callback for body selection (will be set by main window)
        self.on_body_clicked: Optional[Callable[[int], None]] = None

        # Callbacks for face and edge selection
        self.on_face_clicked: Optional[Callable[[int, int], None]] = None  # body_id, face_index
        self.on_edge_clicked: Optional[Callable[[int, int], None]] = None  # body_id, edge_index
        self.on_vertex_clicked: Optional[Callable[[int, int], None]] = None  # body_id, vertex_index

        # Current selection mode
        self.selection_mode: str = "Body"  # "Body", "Face", "Edge", or "Vertex"

    def set_selection_mode(self, mode: str):
        """
        Set the selection mode

        Args:
            mode: "Body", "Face", "Edge", or "Vertex"
        """
        self.selection_mode = mode

        # Update selection mode in OCC context
        ctx = self._display.Context
        ctx.Deactivate()  # Deactivate current mode

        if mode == "Body":
            # Mode 0: select entire shape
            for body_id, ais_shape in self.body_ais_map.items():
                ctx.Activate(ais_shape, 0, False)
        elif mode == "Face":
            # Mode 4: select faces
            for body_id, ais_shape in self.body_ais_map.items():
                ctx.Activate(ais_shape, 4, False)
        elif mode == "Edge":
            # Mode 2: select edges
            for body_id, ais_shape in self.body_ais_map.items():
                ctx.Activate(ais_shape, 2, False)
        elif mode == "Vertex":
            # Mode 1: select vertices
            for body_id, ais_shape in self.body_ais_map.items():
                ctx.Activate(ais_shape, 1, False)

        ctx.UpdateCurrentViewer()
        print(f"Viewer selection mode changed to: {mode}")


    def set_body_mapping(self, body_ais_map: Dict[int, AIS_Shape], bodies_dict: Dict[int, RigidBody] = None):
        """
        Set the mapping between body IDs and AIS shapes for selection

        Args:
            body_ais_map: Dictionary mapping body_id to AIS_Shape
            bodies_dict: Dictionary mapping body_id to RigidBody (for face/edge selection)
        """
        self.body_ais_map = body_ais_map

        # Create reverse mapping
        self.ais_body_map.clear()
        for body_id, ais_shape in body_ais_map.items():
            self.ais_body_map[ais_shape] = body_id

        # Store bodies for face/edge selection
        if bodies_dict:
            self.bodies_dict = bodies_dict

        print(f"Viewer: Body mapping set for {len(self.ais_body_map)} bodies")

        # Re-apply selection mode activation
        self.set_selection_mode(self.selection_mode)

    def mouseReleaseEvent(self, event):
        """
        Handle mouse release events for selection

        Args:
            event: Mouse event
        """
        from PySide6.QtCore import Qt

        pt = event.pos()
        modifiers = event.modifiers()

        # Handle left-click selection ourselves
        if event.button() == Qt.LeftButton:
            if not self._select_area and modifiers == Qt.NoModifier:
                # Single body selection - use our custom logic
                self._select_at_position(pt.x(), pt.y())
                return  # Don't call parent - we handled it

        # For all other cases, use parent's implementation
        super().mouseReleaseEvent(event)

    def _select_at_position(self, x: int, y: int):
        """
        Perform selection at the given screen position using proper OCC selection API

        Args:
            x: Screen x coordinate (pixels on screen)
            y: Screen y coordinate (pixels on screen)
        """
        print(f"\n=== Mouse Click at screen position ({x}, {y}) ===")
        print(f"Selection mode: {self.selection_mode}")
        print(f"Known body mappings: {len(self.ais_body_map)} bodies")

        # Account for high-DPI displays: OCC expects device pixels
        dpr = float(self.devicePixelRatioF())
        px = int(x * dpr)
        py = int(y * dpr)

        # First check what's under the cursor with MoveTo (for debugging)
        self._display.Context.MoveTo(px, py, self._display.View, False)
        if self._display.Context.HasDetected():
            detected = self._display.Context.DetectedInteractive()
            print(f"DEBUG: Detected object under cursor: {detected}")
        else:
            print(f"DEBUG: No object detected under cursor")

        # Use AIS_InteractiveContext selection with an explicit view
        ctx = self._display.Context
        view = self._display.View

        # Clear previous selection so we only process the current click
        ctx.ClearSelected(False)

        # If OCC already detected something under the cursor, select that directly;
        # otherwise fall back to a point pick rectangle.
        if ctx.HasDetected():
            ctx.Select(True)  # Accept the detected owner
        else:
            # Perform a point selection (min/max set to the same pixel)
            # Signature: Select(xMin, yMin, xMax, yMax, view, updateViewer)
            ctx.Select(px, py, px, py, view, False)

        # Check what was selected using Context API
        nb_selected = ctx.NbSelected()
        print(f"DEBUG: Number of objects selected: {nb_selected}")

        ctx.InitSelected()

        if ctx.MoreSelected():
            selected_ais = ctx.SelectedInteractive()
            owner = ctx.SelectedOwner()

            print(f"✓ Selection made!")
            print(f"  Selected AIS object: {selected_ais}")
            print(f"  Selected Owner: {owner}")

            # Check if this is one of our body shapes
            if selected_ais in self.ais_body_map:
                body_id = self.ais_body_map[selected_ais]

                if self.selection_mode == "Body":
                    print(f"✓✓ Body clicked in viewer: Body_{body_id} (ID: {body_id})")
                    if self.on_body_clicked:
                        self.on_body_clicked(body_id)
                    else:
                        print("  WARNING: on_body_clicked callback not set!")

                elif self.selection_mode == "Face":
                    # Extract face index from owner
                    face_index = self._extract_sub_shape_index(owner, TopAbs_FACE)
                    if face_index >= 0:
                        print(f"✓✓ Face clicked in viewer: Body_{body_id}, Face_{face_index}")
                        if self.on_face_clicked:
                            self.on_face_clicked(body_id, face_index)
                        else:
                            print("  WARNING: on_face_clicked callback not set!")
                    else:
                        print("  Could not extract face index")

                elif self.selection_mode == "Edge":
                    # Extract edge index from owner
                    edge_index = self._extract_sub_shape_index(owner, TopAbs_EDGE)
                    if edge_index >= 0:
                        print(f"✓✓ Edge clicked in viewer: Body_{body_id}, Edge_{edge_index}")
                        if self.on_edge_clicked:
                            self.on_edge_clicked(body_id, edge_index)
                        else:
                            print("  WARNING: on_edge_clicked callback not set!")
                    else:
                        print("  Could not extract edge index")
                
                elif self.selection_mode == "Vertex":
                    # Extract vertex index from owner
                    vertex_index = self._extract_sub_shape_index(owner, TopAbs_VERTEX)
                    if vertex_index >= 0:
                        print(f"✓✓ Vertex clicked in viewer: Body_{body_id}, Vertex_{vertex_index}")
                        if self.on_vertex_clicked:
                            self.on_vertex_clicked(body_id, vertex_index)
                        else:
                            print("  WARNING: on_vertex_clicked callback not set!")
                    else:
                        print("  Could not extract vertex index")
            else:
                print("  → This is a non-body object (frame, COM marker, etc.)")
                print(f"  → Available body IDs: {list(self.ais_body_map.values())}")
        else:
            print(f"✗ No selection made - empty space or non-selectable object")

    def _extract_sub_shape_index(self, owner, sub_shape_type) -> int:
        """
        Extract the index of a sub-shape (face or edge) from the selection owner

        Args:
            owner: The selection owner from OCC
            sub_shape_type: TopAbs_FACE or TopAbs_EDGE

        Returns:
            Index of the sub-shape, or -1 if not found
        """
        try:
            from OCC.Core.StdSelect import StdSelect_BRepOwner
            from OCC.Core.TopoDS import topods
            from OCC.Core.TopExp import TopExp_Explorer
            from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE

            # Get the selected shape from the owner
            selected_shape = None
            ctx = self._display.Context

            # Method 1: Ask the Context directly (Preferred)
            if ctx.HasSelectedShape():
                try:
                    selected_shape = ctx.SelectedShape()
                    # print("DEBUG: Got shape from ctx.SelectedShape()")
                except Exception as e:
                    print(f"DEBUG: ctx.SelectedShape() failed: {e}")

            # Method 2: Try getting it from the owner (Fallback)
            if selected_shape is None:
                # Try different ways to get the shape from owner
                if hasattr(owner, 'Shape'):
                    try:
                        selected_shape = owner.Shape()
                    except Exception as e:
                        print(f"DEBUG: owner.Shape() failed: {e}")

            if selected_shape is None:
                # Try casting to BRepOwner
                try:
                    brep_owner = StdSelect_BRepOwner.DownCast(owner)
                    if brep_owner is not None and not brep_owner.IsNull():
                        selected_shape = brep_owner.Shape()
                    else:
                        print(f"DEBUG: StdSelect_BRepOwner.DownCast(owner) returned None/Null for owner type: {type(owner)}")
                except Exception as e:
                    print(f"DEBUG: casting to BRepOwner failed: {e}")

            if selected_shape is None or selected_shape.IsNull():
                print(f"DEBUG: Could not get shape from owner: {owner}")
                return -1

            # Get the currently selected parent AIS shape
            parent_ais = None
            parent_body_id = None

            ctx.InitSelected()
            if ctx.MoreSelected():
                parent_ais = ctx.SelectedInteractive()
                
                # Check mapping using direct lookup first
                if parent_ais in self.ais_body_map:
                    parent_body_id = self.ais_body_map[parent_ais]
                else:
                    # Fallback: Compare pointers/handles
                    # print("DEBUG: Direct AIS lookup failed, trying equality check")
                    for ais_shape, body_id in self.ais_body_map.items():
                        if ais_shape.IsEqual(parent_ais) or ais_shape == parent_ais:
                            parent_body_id = body_id
                            # print(f"DEBUG: Found parent body {body_id} via equality check")
                            break

            if parent_body_id is None or parent_body_id not in self.bodies_dict:
                print(f"DEBUG: Parent body not found for AIS: {parent_ais}")
                return -1

            body = self.bodies_dict[parent_body_id]

            # Now match the selected shape to get its index
            if sub_shape_type == TopAbs_FACE:
                try:
                    selected_face = topods.Face(selected_shape)
                    if selected_face.IsNull():
                        print(f"DEBUG: Selected shape is not a valid face")
                        return -1

                    # Count faces and find a match
                    explorer = TopExp_Explorer(body.shape, TopAbs_FACE)
                    face_index = 0

                    while explorer.More():
                        current_shape = explorer.Current()
                        current_face = topods.Face(current_shape)

                        # Method 1: IsEqual (checks TShape and Orientation)
                        if current_face.IsEqual(selected_face):
                            print(f"DEBUG: Found face match using IsEqual at index {face_index}")
                            return face_index
                        
                        # Method 2: Check TShape pointer directly
                        if current_face.TShape() == selected_face.TShape():
                            print(f"DEBUG: Found face match using TShape at index {face_index}")
                            return face_index

                        # Method 3: Hash comparison using Python's hash()
                        try:
                            if hash(current_face) == hash(selected_face):
                                print(f"DEBUG: Found face match using hash() at index {face_index}")
                                return face_index
                        except TypeError:
                            pass  # hash not supported, skip this method

                        face_index += 1
                        explorer.Next()

                    print(f"DEBUG: No match found for face")
                    return -1

                except Exception as e:
                    print(f"DEBUG: Error processing face: {e}")
                    return -1

            elif sub_shape_type == TopAbs_EDGE:
                try:
                    selected_edge = topods.Edge(selected_shape)
                    if selected_edge.IsNull():
                        print(f"DEBUG: Selected shape is not a valid edge")
                        return -1

                    # Count edges and find a match
                    explorer = TopExp_Explorer(body.shape, TopAbs_EDGE)
                    edge_index = 0

                    while explorer.More():
                        current_shape = explorer.Current()
                        current_edge = topods.Edge(current_shape)

                        # Method 1: IsEqual
                        if current_edge.IsEqual(selected_edge):
                            print(f"DEBUG: Found edge match using IsEqual at index {edge_index}")
                            return edge_index

                        # Method 2: TShape
                        if current_edge.TShape() == selected_edge.TShape():
                            print(f"DEBUG: Found edge match using TShape at index {edge_index}")
                            return edge_index
                            
                        # Method 3: Hash comparison using Python's hash()
                        try:
                            if hash(current_edge) == hash(selected_edge):
                                print(f"DEBUG: Found edge match using hash() at index {edge_index}")
                                return edge_index
                        except TypeError:
                            pass  # hash not supported, skip this method

                        edge_index += 1
                        explorer.Next()

                    print(f"DEBUG: No match found for edge")
                    return -1

                except Exception as e:
                    print(f"DEBUG: Error processing edge: {e}")
                    return -1

            elif sub_shape_type == TopAbs_VERTEX:
                try:
                    selected_vertex = topods.Vertex(selected_shape)
                    if selected_vertex.IsNull():
                        print(f"DEBUG: Selected shape is not a valid vertex")
                        return -1

                    # Count vertices and find a match
                    explorer = TopExp_Explorer(body.shape, TopAbs_VERTEX)
                    vertex_index = 0

                    while explorer.More():
                        current_shape = explorer.Current()
                        current_vertex = topods.Vertex(current_shape)

                        # Method 1: IsEqual
                        if current_vertex.IsEqual(selected_vertex):
                            print(f"DEBUG: Found vertex match using IsEqual at index {vertex_index}")
                            return vertex_index

                        # Method 2: TShape
                        if current_vertex.TShape() == selected_vertex.TShape():
                            print(f"DEBUG: Found vertex match using TShape at index {vertex_index}")
                            return vertex_index
                            
                        # Method 3: Hash comparison using Python's hash()
                        try:
                            if hash(current_vertex) == hash(selected_vertex):
                                print(f"DEBUG: Found vertex match using hash() at index {vertex_index}")
                                return vertex_index
                        except TypeError:
                            pass  # hash not supported, skip this method

                        vertex_index += 1
                        explorer.Next()

                    print(f"DEBUG: No match found for vertex")
                    return -1

                except Exception as e:
                    print(f"DEBUG: Error processing vertex: {e}")
                    return -1

        except Exception as e:
            print(f"Error extracting sub-shape index: {e}")
            import traceback
            traceback.print_exc()
            return -1

        return -1

    def clear_mappings(self):
        """Clear all body-to-AIS mappings"""
        self.body_ais_map.clear()
        self.ais_body_map.clear()
    
    def remove_body_from_mapping(self, body_id: int):
        """
        Remove a body from the selection mappings
        
        Args:
            body_id: ID of the body to remove
        """
        if body_id in self.body_ais_map:
            ais_shape = self.body_ais_map[body_id]
            
            # Remove from both mappings
            del self.body_ais_map[body_id]
            if ais_shape in self.ais_body_map:
                del self.ais_body_map[ais_shape]
            
            # Remove from bodies dict
            if body_id in self.bodies_dict:
                del self.bodies_dict[body_id]
            
            print(f"Body {body_id} removed from viewer mappings")
