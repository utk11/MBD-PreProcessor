

import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                QFileDialog, QMenuBar, QMessageBox, QSplitter, QDialog)
from PySide6.QtCore import Qt
from PySide6.QtGui import QAction
from typing import List, Optional, Dict, Tuple
import numpy as np

# PythonOCC imports for 3D visualization
from OCC.Display.backend import load_backend
load_backend('pyside6')

# Import core modules
from core.step_parser import StepParser
from core.data_structures import RigidBody, Frame, Joint, JointType, Force, Torque, MotorType
from core.physics_calculator import PhysicsCalculator
from core.geometry_utils import GeometryUtils, FaceProperties, EdgeProperties, VertexProperties

# Import GUI modules
from gui.body_tree_widget import BodyTreeWidget
from gui.property_panel import PropertyPanel
from gui.viewer_3d import SelectableViewer3d
from gui.joint_dialog import JointCreationDialog
from gui.force_dialog import ForceDialog
from gui.torque_dialog import TorqueDialog
from gui.motor_dialog import MotorDialog

# Import visualization modules
from visualization.body_renderer import BodyRenderer
from visualization.frame_renderer import FrameRenderer
from visualization.face_renderer import FaceRenderer
from visualization.edge_renderer import EdgeRenderer
from visualization.vertex_renderer import VertexRenderer
from visualization.joint_renderer import JointRenderer
from visualization.force_renderer import ForceRenderer
from visualization.torque_renderer import TorqueRenderer
from visualization.motor_renderer import MotorRenderer

# Import export module
from export.exporter import AssemblyExporter


class MainWindow(QMainWindow):
    """Main application window with 3D viewer"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Body Dynamics Preprocessor")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create menu bar
        self.create_menu_bar()
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create main horizontal splitter (left + middle + right)
        main_splitter = QSplitter(Qt.Horizontal)
        
        # Create body tree widget (left panel)
        self.body_tree = BodyTreeWidget()
        main_splitter.addWidget(self.body_tree)
        
        # Create 3D viewer (middle panel) with click selection
        self.viewer_3d = SelectableViewer3d(central_widget)
        main_splitter.addWidget(self.viewer_3d)
        
        # Create property panel (right panel)
        self.property_panel = PropertyPanel()
        main_splitter.addWidget(self.property_panel)
        
        # Set splitter proportions: 15% left, 65% viewer, 20% right
        main_splitter.setStretchFactor(0, 15)  # Left panel (15%)
        main_splitter.setStretchFactor(1, 65)  # Viewer (65%)
        main_splitter.setStretchFactor(2, 20)  # Right panel (20%)
        
        # Add splitter to layout
        layout.addWidget(main_splitter)
        
        # Initialize the display
        self.display = self.viewer_3d._display
        
        # Initialize body storage
        self.bodies: List[RigidBody] = []
        
        # Initialize renderers
        self.body_renderer = BodyRenderer(self.display)
        self.frame_renderer = FrameRenderer(self.display)
        self.face_renderer = FaceRenderer(self.display)
        self.edge_renderer = EdgeRenderer(self.display)
        self.vertex_renderer = VertexRenderer(self.display)
        self.joint_renderer = JointRenderer(self.display)
        self.force_renderer = ForceRenderer(self.display)
        self.torque_renderer = TorqueRenderer(self.display)
        self.motor_renderer = MotorRenderer(self.display)
        
        # Create world frame at origin
        self.world_frame = Frame(name="World Frame")
        self.frame_renderer.render_frame(self.world_frame, visible=True)
        
        # Create Ground Body (ID -1)
        self.ground_body = RigidBody(-1, None, "Ground")
        self.ground_body.center_of_mass = [0.0, 0.0, 0.0]
        # Ground's local frame is effectively the world frame
        self.ground_body.local_frame = self.world_frame
        
        # Connect tree widget signals
        self.body_tree.body_selected.connect(self.on_body_selected)
        self.body_tree.frame_selected.connect(self.on_frame_selected)
        self.body_tree.delete_frame_requested.connect(self.on_frame_deleted)
        self.body_tree.delete_body_requested.connect(self.on_body_deleted)
        self.body_tree.delete_multiple_bodies_requested.connect(self.on_multiple_bodies_deleted)
        self.body_tree.isolate_body_requested.connect(self.on_isolate_body)
        self.body_tree.exit_isolation_requested.connect(self.on_exit_isolation)
        
        # Connect viewer click selection
        self.viewer_3d.on_body_clicked = self.on_body_clicked_in_viewer
        
        # Connect property panel signals
        self.property_panel.com_visibility_changed.connect(self.on_com_visibility_changed)
        self.property_panel.world_frame_visibility_changed.connect(self.on_world_frame_visibility_changed)
        self.property_panel.local_frame_visibility_changed.connect(self.on_local_frame_visibility_changed)
        self.property_panel.frame_visibility_changed.connect(self.on_frame_visibility_changed)
        self.property_panel.frame_highlight_changed.connect(self.on_frame_highlight_changed)
        self.property_panel.all_frames_visibility_changed.connect(self.on_all_frames_visibility_changed)
        self.property_panel.create_frame_from_face.connect(self.on_create_frame_from_face)
        self.property_panel.create_frame_from_edge.connect(self.on_create_frame_from_edge)
        self.property_panel.create_frame_from_vertex.connect(self.on_create_frame_from_vertex)
        self.property_panel.frame_position_changed.connect(self.on_frame_position_changed)
        self.property_panel.frame_rotation_changed.connect(self.on_frame_rotation_changed)
        self.property_panel.body_visibility_changed.connect(self.on_body_visibility_changed)
        self.property_panel.contact_detection_changed.connect(self.on_contact_detection_changed)
        
        # Track currently selected body for local frame rendering
        self.selected_body_id: Optional[int] = None

        # Track last face selection for frame creation
        self.last_face_selection: Optional[Tuple[int, int]] = None  # (body_id, face_index)
        
        # Track last edge selection for frame creation
        self.last_edge_selection: Optional[Tuple[int, int]] = None  # (body_id, edge_index)
        
        # Track last vertex selection for frame creation
        self.last_vertex_selection: Optional[Tuple[int, int]] = None  # (body_id, vertex_index)

        # Store face and edge properties for current bodies
        self.face_properties_map: Dict[int, List[FaceProperties]] = {}  # body_id -> list of FaceProperties
        self.edge_properties_map: Dict[int, List[EdgeProperties]] = {}  # body_id -> list of EdgeProperties
        self.vertex_properties_map: Dict[int, List[VertexProperties]] = {}  # body_id -> list of VertexProperties

        # Store user-created frames (name -> Frame)
        self.created_frames: Dict[str, Frame] = {}
        
        # Frame-to-body association tracking (replaces fragile string matching)
        self.frame_to_body_map: Dict[str, int] = {}  # frame_name -> body_id
        
        # Store joints (name -> Joint)
        self.joints: Dict[str, Joint] = {}
        
        # Store forces (name -> Force)
        self.forces: Dict[str, Force] = {}
        
        # Store torques (name -> Torque)
        self.torques: Dict[str, Torque] = {}
        
        # Isolation mode state
        self.isolation_active: bool = False
        self.isolated_body_id: Optional[int] = None
        
        # Connect property panel selection mode changes
        self.property_panel.selection_mode_changed.connect(self.on_selection_mode_changed)
        
        # Connect viewer face and edge click selection
        self.viewer_3d.on_face_clicked = self.on_face_clicked_in_viewer
        self.viewer_3d.on_edge_clicked = self.on_edge_clicked_in_viewer
        self.viewer_3d.on_vertex_clicked = self.on_vertex_clicked_in_viewer

        # Connect tree joint selection
        self.body_tree.joint_selected.connect(self.on_joint_selected)
        self.body_tree.delete_joint_requested.connect(self.on_joint_deleted)
        
        # Connect tree force selection
        self.body_tree.force_selected.connect(self.on_force_selected)
        self.body_tree.delete_force_requested.connect(self.on_force_deleted)
        
        # Connect tree torque selection
        self.body_tree.torque_selected.connect(self.on_torque_selected)
        self.body_tree.delete_torque_requested.connect(self.on_torque_deleted)
        
        # Store unit scale
        self.unit_scale = 1.0
        
        # Track current STEP file path for project saving
        self.current_step_file: Optional[str] = None

        print("Application initialized successfully!")
        print("Viewer ready. Use File > Open to load a STEP file.")
        print("Mouse controls: Right-click drag to rotate, Shift+Right-click to pan, Wheel to zoom.")
    
    def create_menu_bar(self):
        """Create the menu bar with File menu"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("File")

        # Open action
        open_action = QAction("Open STEP File...", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self.open_step_file)
        file_menu.addAction(open_action)
        
        # Save Project action
        save_project_action = QAction("Save Project...", self)
        save_project_action.setShortcut("Ctrl+S")
        save_project_action.triggered.connect(self.save_project)
        file_menu.addAction(save_project_action)
        
        # Load Project action
        load_project_action = QAction("Load Project...", self)
        load_project_action.setShortcut("Ctrl+L")
        load_project_action.triggered.connect(self.load_project)
        file_menu.addAction(load_project_action)

        # Separator
        file_menu.addSeparator()

        # Exit action
        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Assembly menu
        assembly_menu = menubar.addMenu("Assembly")
        
        create_joint_action = QAction("Create Joint...", self)
        create_joint_action.triggered.connect(self.create_joint)
        assembly_menu.addAction(create_joint_action)
        
        assembly_menu.addSeparator()
        
        add_motor_action = QAction("Add Motor to Joint...", self)
        add_motor_action.triggered.connect(self.add_motor_to_joint)
        assembly_menu.addAction(add_motor_action)
        
        remove_motor_action = QAction("Remove Motor from Joint...", self)
        remove_motor_action.triggered.connect(self.remove_motor_from_joint)
        assembly_menu.addAction(remove_motor_action)
        
        assembly_menu.addSeparator()
        
        add_force_action = QAction("Add Force...", self)
        add_force_action.triggered.connect(self.add_force)
        assembly_menu.addAction(add_force_action)
        
        add_torque_action = QAction("Add Torque...", self)
        add_torque_action.triggered.connect(self.add_torque)
        assembly_menu.addAction(add_torque_action)

        # Export menu
        export_menu = menubar.addMenu("Export")
        
        export_json_action = QAction("Export Assembly as JSON...", self)
        export_json_action.setShortcut("Ctrl+E")
        export_json_action.triggered.connect(self.export_assembly_json)
        export_menu.addAction(export_json_action)
        
        export_obj_action = QAction("Export Body Meshes as OBJ...", self)
        export_obj_action.setShortcut("Ctrl+Shift+E")
        export_obj_action.triggered.connect(self.export_body_meshes_obj)
        export_menu.addAction(export_obj_action)

        # Debug menu
        debug_menu = menubar.addMenu("Debug")
        
        # Test Joint action
        test_joint_action = QAction("Create Test Joint", self)
        test_joint_action.triggered.connect(self.create_test_joint)
        debug_menu.addAction(test_joint_action)

    def open_step_file(self):
        """Open file dialog and load selected STEP file"""
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "Open STEP File",
            "",
            "STEP Files (*.step *.stp);;All Files (*.*)"
        )

        if filepath:
            self.load_step_file(filepath)

    def load_step_file(self, filepath):
        """Load and display a STEP file"""
        try:
            print(f"Loading STEP file: {filepath}")
            
            # Store the current STEP file path
            self.current_step_file = filepath

            # Clear previous display and bodies
            self.body_renderer.clear_all()
            self.face_renderer.clear_highlight()
            self.edge_renderer.clear_highlight()
            self.vertex_renderer.clear_highlight()
            self.viewer_3d.clear_mappings()
            self.bodies.clear()
            self.body_tree.clear()
            self.property_panel.clear()
            self.frame_renderer.clear_all_frames()
            self.created_frames.clear()
            self.frame_to_body_map.clear()  # Clear frame-to-body associations
            self.joints.clear()
            self.joint_renderer.clear()
            self.forces.clear()
            self.force_renderer.clear_all()
            self.torques.clear()
            self.torque_renderer.clear_all_torques()
            self.last_face_selection = None
            self.last_edge_selection = None
            self.last_vertex_selection = None
            
            # Reset selection mode to Body (default)
            self.property_panel.set_selection_mode("Body")

            # Read the STEP file using StepParser
            shape, unit_scale = StepParser.load_step_file(filepath)
            self.unit_scale = unit_scale

            # Update renderers with the new unit scale
            self.frame_renderer.set_unit_scale(unit_scale)
            self.body_renderer.set_unit_scale(unit_scale)
            self.force_renderer.set_unit_scale(unit_scale)
            self.torque_renderer.set_unit_scale(unit_scale)
            self.vertex_renderer.set_unit_scale(unit_scale)

            if shape is None:
                QMessageBox.warning(
                    self,
                    "Error",
                    "Could not load STEP file. Please check the file format."
                )
                print("Error: Could not load STEP file")
                return

            # Extract individual bodies from the shape
            self.bodies = StepParser.extract_bodies_from_compound(shape)

            print(f"Found {len(self.bodies)} body/bodies in the assembly:")
            for body in self.bodies:
                print(f"  - {body.name} (ID: {body.id})")

            # Calculate volumes for all bodies
            print("\nCalculating volumes...")
            PhysicsCalculator.calculate_volumes_for_bodies(self.bodies, unit_scale)
            print("Volume calculation complete.\n")

            # Calculate centers of mass for all bodies
            PhysicsCalculator.calculate_centers_of_mass_for_bodies(self.bodies, unit_scale)

            # Calculate inertia tensors for all bodies
            PhysicsCalculator.calculate_inertia_tensors_for_bodies(self.bodies, unit_scale)

            # Initialize local frames for all bodies
            PhysicsCalculator.initialize_local_frames(self.bodies)

            # Update the body tree widget
            self.body_tree.update_bodies(self.bodies)

            # Display all bodies in the viewer using the renderer
            self.body_renderer.display_bodies(self.bodies)

            # Extract face and edge properties for all bodies
            print("\nExtracting face, edge, and vertex properties...")
            self.face_properties_map.clear()
            self.edge_properties_map.clear()
            self.vertex_properties_map.clear()
            bodies_dict = {}  # body_id -> RigidBody

            for body in self.bodies:
                self.face_properties_map[body.id] = GeometryUtils.extract_faces(body.shape, self.unit_scale)
                self.edge_properties_map[body.id] = GeometryUtils.extract_edges(body.shape, self.unit_scale)
                self.vertex_properties_map[body.id] = GeometryUtils.extract_vertices(body.shape, self.unit_scale)
                bodies_dict[body.id] = body
                print(f"Body {body.id}: {len(self.face_properties_map[body.id])} faces, "
                      f"{len(self.edge_properties_map[body.id])} edges, "
                      f"{len(self.vertex_properties_map[body.id])} vertices")

            # Update viewer's body-to-AIS mapping for click selection (with bodies_dict for face/edge selection)
            self.viewer_3d.set_body_mapping(self.body_renderer.body_ais_shapes, bodies_dict)

            # Fit view
            self.display.FitAll()
            
            # Scale world frame axes based on bounding box
            # Get bounding box dimensions by computing from displayed shapes
            from OCC.Core.Bnd import Bnd_Box
            from OCC.Core.BRepBndLib import brepbndlib

            bbox = Bnd_Box()
            for body in self.bodies:
                brepbndlib.Add(body.shape, bbox)

            if not bbox.IsVoid():
                xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
                x_range = xmax - xmin
                y_range = ymax - ymin
                z_range = zmax - zmin
                max_dimension = max(x_range, y_range, z_range)

                # Scale axes to 20% of model size
                axis_scale = max_dimension * 0.2
                self.frame_renderer.set_axis_scale(axis_scale)
                
                # Scale forces to 30% of model size (increased for visibility)
                force_scale = max_dimension * 0.3
                self.force_renderer.set_force_scale(force_scale)
                
                # Scale torques to 20% of model size
                torque_scale = max_dimension * 0.2
                self.torque_renderer.set_torque_scale(torque_scale)

                # Re-render world frame with new scale
                current_visibility = self.frame_renderer.frame_visible.get("World Frame", True)
                self.frame_renderer.render_frame(self.world_frame, visible=current_visibility)

                print(f"Frame axes scaled to {axis_scale:.4f}")
                print(f"Force arrows scaled to {force_scale:.4f}") 
                print(f"Torque circles scaled to {torque_scale:.4f}")

            print(f"STEP file loaded successfully! Total bodies: {len(self.bodies)}")

        except FileNotFoundError as e:
            QMessageBox.critical(
                self,
                "File Not Found",
                str(e)
            )
            print(f"File not found: {e}")
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error",
                f"Error loading STEP file:\n{str(e)}"
            )
            print(f"Error loading STEP file: {e}")

    def on_body_clicked_in_viewer(self, body_id: int):
        """
        Handle body click selection from the 3D viewer

        Args:
            body_id: ID of the clicked body
        """
        # Update tree selection to match clicked body
        self.body_tree.select_body(body_id)
        # Tree selection will trigger on_body_selected automatically

    def on_body_selected(self, body_id: int):
        """
        Handle body selection from the tree widget

        Args:
            body_id: ID of the selected body
        """
        print(f"Highlighting body {body_id} in viewer")
        self.body_renderer.highlight_body(body_id)

        # Ensure the property panel is showing the body section
        self.property_panel.set_selection_mode("Body")
        
        # Hide previous body's local frame if any
        if self.selected_body_id is not None:
            prev_body = next((b for b in self.bodies if b.id == self.selected_body_id), None)
            if prev_body and prev_body.local_frame:
                self.frame_renderer.remove_frame(prev_body.local_frame.name)
        
        # Update selected body ID
        self.selected_body_id = body_id
        
        # Find the body and update property panel
        selected_body = None
        for body in self.bodies:
            if body.id == body_id:
                selected_body = body
                break
        
        if selected_body:
            self.property_panel.show_body_properties(selected_body)
            
            # Show local frame if it exists and checkbox is enabled
            if selected_body.local_frame:
                is_visible = self.property_panel.local_frame_checkbox.isChecked()
                self.frame_renderer.render_frame(selected_body.local_frame, visible=is_visible)

    def on_com_visibility_changed(self, visible: bool):
        """
        Handle COM visibility toggle from property panel

        Args:
            visible: True to show COM marker, False to hide it
        """
        self.body_renderer.set_com_visibility(visible)

    def on_world_frame_visibility_changed(self, visible: bool):
        """
        Handle world frame visibility toggle from property panel

        Args:
            visible: True to show world frame, False to hide it
        """
        self.frame_renderer.set_frame_visibility("World Frame", visible)

    def on_local_frame_visibility_changed(self, visible: bool):
        """
        Handle local frame visibility toggle from property panel

        Args:
            visible: True to show local frame, False to hide it
        """
        if self.selected_body_id == -1: # Ground
             if self.ground_body.local_frame:
                 self.frame_renderer.set_frame_visibility(self.ground_body.local_frame.name, visible)
             return

        if self.selected_body_id is not None:
            # Find the selected body
            selected_body = next((b for b in self.bodies if b.id == self.selected_body_id), None)
            if selected_body and selected_body.local_frame:
                self.frame_renderer.set_frame_visibility(selected_body.local_frame.name, visible)

    def on_frame_visibility_changed(self, frame_name: str, visible: bool):
        """
        Handle frame visibility toggle from property panel
        
        Args:
            frame_name: Name of the frame
            visible: True to show frame, False to hide it
        """
        self.frame_renderer.set_frame_visibility(frame_name, visible)
        print(f"Frame '{frame_name}' visibility set to {visible}")
    
    def on_frame_highlight_changed(self, frame_name: str, highlighted: bool):
        """
        Handle frame highlight toggle from property panel
        
        Args:
            frame_name: Name of the frame
            highlighted: True to highlight frame, False to unhighlight
        """
        self.frame_renderer.highlight_frame(frame_name, highlighted)
        print(f"Frame '{frame_name}' highlight set to {highlighted}")
    
    def on_all_frames_visibility_changed(self, visible: bool):
        """
        Handle all frames visibility toggle from property panel
        
        Args:
            visible: True to show all frames, False to hide all frames
        """
        self.frame_renderer.set_all_frames_visibility(visible)
        print(f"All frames visibility set to {visible}")

    def on_selection_mode_changed(self, mode: str):
        """
        Handle selection mode change from property panel
        
        Args:
            mode: "Body", "Face", "Edge", or "Vertex"
        """
        self.viewer_3d.set_selection_mode(mode)
        print(f"Selection mode changed to: {mode}")
        
        # Clear specific highlights when switching modes
        if mode == "Body":
            self.face_renderer.clear_highlight()
            self.edge_renderer.clear_highlight()
            self.vertex_renderer.clear_highlight()
        elif mode == "Face":
            # Keep body highlight? Usually yes, to see context. 
            # But clear edge and vertex highlight.
            self.edge_renderer.clear_highlight()
            self.vertex_renderer.clear_highlight()
        elif mode == "Edge":
            # Clear face and vertex highlight.
            self.face_renderer.clear_highlight()
            self.vertex_renderer.clear_highlight()
        elif mode == "Vertex":
            # Clear face and edge highlight.
            self.face_renderer.clear_highlight()
            self.edge_renderer.clear_highlight()
    
    def on_face_clicked_in_viewer(self, body_id: int, face_index: int):
        """
        Handle face click selection from the 3D viewer
        
        Args:
            body_id: ID of the body containing the face
            face_index: Index of the clicked face
        """
        print(f"Face clicked: Body {body_id}, Face {face_index}")
        
        # Get face properties
        if body_id in self.face_properties_map and face_index < len(self.face_properties_map[body_id]):
            face_props = self.face_properties_map[body_id][face_index]
            self.property_panel.set_selection_mode("Face")
            self.property_panel.show_face_properties(face_props)
            self.last_face_selection = (body_id, face_index)
            
            # Highlight the face
            self.face_renderer.highlight_face(face_props.face)
        else:
            print(f"Face properties not found for body {body_id}, face {face_index}")
            self.last_face_selection = None
            self.face_renderer.clear_highlight()
    
    def on_edge_clicked_in_viewer(self, body_id: int, edge_index: int):
        """
        Handle edge click selection from the 3D viewer 
        
        Args:
            body_id: ID of the body containing the edge
            edge_index: Index of the clicked edge
        """
        print(f"Edge clicked: Body {body_id}, Edge {edge_index}")
        
        # Get edge properties
        if body_id in self.edge_properties_map and edge_index < len(self.edge_properties_map[body_id]):
            edge_props = self.edge_properties_map[body_id][edge_index]
            self.property_panel.set_selection_mode("Edge")
            self.property_panel.show_edge_properties(edge_props)
            self.last_edge_selection = (body_id, edge_index)
            
            # Highlight the edge
            self.edge_renderer.highlight_edge(edge_props.edge)
        else:
            print(f"Edge properties not found for body {body_id}, edge {edge_index}")
            self.last_edge_selection = None
            self.edge_renderer.clear_highlight()
    
    def on_vertex_clicked_in_viewer(self, body_id: int, vertex_index: int):
        """
        Handle vertex click selection from the 3D viewer
        
        Args:
            body_id: ID of the body containing the vertex
            vertex_index: Index of the clicked vertex
        """
        print(f"Vertex clicked: Body {body_id}, Vertex {vertex_index}")
        
        # Get vertex properties
        if body_id in self.vertex_properties_map and vertex_index < len(self.vertex_properties_map[body_id]):
            vertex_props = self.vertex_properties_map[body_id][vertex_index]
            self.property_panel.set_selection_mode("Vertex")
            self.property_panel.show_vertex_properties(vertex_props)
            self.last_vertex_selection = (body_id, vertex_index)
            
            # Highlight the vertex
            self.vertex_renderer.highlight_vertex(vertex_props.vertex)
        else:
            print(f"Vertex properties not found for body {body_id}, vertex {vertex_index}")
            self.last_vertex_selection = None
            self.vertex_renderer.clear_highlight()

    def on_create_frame_from_face(self):
        """Create and render a frame from the last selected face"""
        if not self.last_face_selection:
            QMessageBox.information(self, "No Face Selected", "Select a face first to create a frame.")
            return
        body_id, face_index = self.last_face_selection

        if body_id not in self.face_properties_map or face_index >= len(self.face_properties_map[body_id]):
            QMessageBox.warning(self, "Face Missing", "Face properties are unavailable for the current selection.")
            return

        face_props = self.face_properties_map[body_id][face_index]

        base_name = f"Frame_B{body_id}_F{face_index}"
        frame_name = base_name
        suffix = 1
        while frame_name in self.created_frames:
            frame_name = f"{base_name}_{suffix}"
            suffix += 1

        frame = GeometryUtils.frame_from_face(face_props, name=frame_name, unit_scale=self.unit_scale)
        
        self.created_frames[frame_name] = frame
        self.frame_to_body_map[frame_name] = body_id  # Track frame-to-body association
        self.frame_renderer.render_frame(frame, visible=True)
        
        # Update frame tree and select new frame
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)
        
        self.property_panel.show_frame_properties(frame)
        print(f"Created frame '{frame_name}' from face {face_index} on body {body_id}")

    def on_create_frame_from_edge(self):
        """Create and render a frame from the last selected edge"""
        if not self.last_edge_selection:
            QMessageBox.information(self, "No Edge Selected", "Select an edge first to create a frame.")
            return
        body_id, edge_index = self.last_edge_selection

        if body_id not in self.edge_properties_map or edge_index >= len(self.edge_properties_map[body_id]):
            QMessageBox.warning(self, "Edge Missing", "Edge properties are unavailable for the current selection.")
            return

        edge_props = self.edge_properties_map[body_id][edge_index]

        base_name = f"Frame_B{body_id}_E{edge_index}"
        frame_name = base_name
        suffix = 1
        while frame_name in self.created_frames:
            frame_name = f"{base_name}_{suffix}"
            suffix += 1

        frame = GeometryUtils.frame_from_edge(edge_props, name=frame_name, unit_scale=self.unit_scale)
        
        self.created_frames[frame_name] = frame
        self.frame_to_body_map[frame_name] = body_id  # Track frame-to-body association
        self.frame_renderer.render_frame(frame, visible=True)
        
        # Update frame tree and select new frame
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)
        
        self.property_panel.show_frame_properties(frame)
        print(f"Created frame '{frame_name}' from edge {edge_index} on body {body_id}")
    
    def on_create_frame_from_vertex(self):
        """Create and render a frame from the last selected vertex"""
        if not self.last_vertex_selection:
            QMessageBox.information(self, "No Vertex Selected", "Select a vertex first to create a frame.")
            return
        body_id, vertex_index = self.last_vertex_selection

        if body_id not in self.vertex_properties_map or vertex_index >= len(self.vertex_properties_map[body_id]):
            QMessageBox.warning(self, "Vertex Missing", "Vertex properties are unavailable for the current selection.")
            return

        vertex_props = self.vertex_properties_map[body_id][vertex_index]

        base_name = f"Frame_B{body_id}_V{vertex_index}"
        frame_name = base_name
        suffix = 1
        while frame_name in self.created_frames:
            frame_name = f"{base_name}_{suffix}"
            suffix += 1

        frame = GeometryUtils.frame_from_vertex(vertex_props, name=frame_name)
        
        self.created_frames[frame_name] = frame
        # Track frame-to-body association if we have a selected body
        if self.selected_body_id is not None:
            self.frame_to_body_map[frame_name] = self.selected_body_id
        self.frame_renderer.render_frame(frame, visible=True)
        
        # Update frame tree and select new frame
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)
        
        self.property_panel.show_frame_properties(frame)
        print(f"Created frame '{frame_name}' from vertex {vertex_index} on body {body_id}")
    
    def on_frame_selected(self, frame_name: str):
        """Handle frame selection from tree"""
        print(f"Frame selected: {frame_name}")
        if frame_name in self.created_frames:
            frame = self.created_frames[frame_name]
            self.property_panel.show_frame_properties(frame)

    def on_frame_deleted(self, frame_name: str):
        """Handle frame deletion request"""
        print(f"Deleting frame: {frame_name}")
        if frame_name in self.created_frames:
            # Remove from Renderer
            self.frame_renderer.remove_frame(frame_name)
            # Remove from storage
            del self.created_frames[frame_name]
            # Update tree
            self.body_tree.update_frames(self.created_frames.values())
            # Clear property panel
            self.property_panel.show_no_selection()
    
    def on_body_deleted(self, body_id: int):
        """Handle body deletion request"""
        print(f"Deleting body: {body_id}")
        
        # Find the body
        body_to_delete = None
        for body in self.bodies:
            if body.id == body_id:
                body_to_delete = body
                break
        
        if body_to_delete is None:
            print(f"Body {body_id} not found")
            return
        
        # Show confirmation dialog
        from PySide6.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            "Delete Body",
            f"Are you sure you want to delete '{body_to_delete.name}'?\n\nThis will also delete:\n" +
            "- Associated frames\n" +
            "- Joints connected to this body",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # Delete joints connected to this body
        joints_to_delete = []
        for joint_name, joint in self.joints.items():
            if joint.body1_id == body_id or joint.body2_id == body_id:
                joints_to_delete.append(joint_name)
        
        for joint_name in joints_to_delete:
            print(f"  Removing joint: {joint_name}")
            self.joint_renderer.remove_joint(joint_name)
            del self.joints[joint_name]
        
        # Delete frames associated with this body (using robust dictionary-based tracking)
        frames_to_delete = [name for name, bid in self.frame_to_body_map.items() if bid == body_id]
        
        for frame_name in frames_to_delete:
            print(f"  Removing frame: {frame_name}")
            self.frame_renderer.remove_frame(frame_name)
            del self.created_frames[frame_name]
            del self.frame_to_body_map[frame_name]  # Clean up association
        
        # Remove body's local frame
        if body_to_delete.local_frame:
            self.frame_renderer.remove_frame(body_to_delete.local_frame.name)
        
        # Remove from renderer
        self.body_renderer.remove_body(body_id)
        
        # Remove from viewer mappings
        self.viewer_3d.remove_body_from_mapping(body_id)
        
        # Remove face and edge properties
        if body_id in self.face_properties_map:
            del self.face_properties_map[body_id]
        if body_id in self.edge_properties_map:
            del self.edge_properties_map[body_id]
        
        # Remove from bodies list
        self.bodies = [b for b in self.bodies if b.id != body_id]
        
        # Clear selection if this was the selected body
        if self.selected_body_id == body_id:
            self.selected_body_id = None
            self.property_panel.show_no_selection()
        
        # Update GUI
        self.body_tree.update_bodies(self.bodies)
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.update_joints_list(self.joints.values())
        
        # Update display
        self.display.Repaint()
        
        print(f"Body {body_id} deleted successfully")
    
    def on_multiple_bodies_deleted(self, body_ids: List[int]):
        """Handle multiple body deletion request"""
        print(f"Deleting {len(body_ids)} bodies: {body_ids}")
        
        # Find all bodies to delete
        bodies_to_delete = [body for body in self.bodies if body.id in body_ids]
        
        if not bodies_to_delete:
            print("No bodies found to delete")
            return
        
        # Show confirmation dialog
        from PySide6.QtWidgets import QMessageBox
        body_names = "\n".join([f"  - {body.name}" for body in bodies_to_delete])
        reply = QMessageBox.question(
            self,
            "Delete Multiple Bodies",
            f"Are you sure you want to delete {len(bodies_to_delete)} bodies?\n\n{body_names}\n\nThis will also delete:\n" +
            "- Associated frames\n" +
            "- Joints connected to these bodies\n" +
            "- Forces and torques on these bodies",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # Process each body deletion
        for body_id in body_ids:
            body_to_delete = next((b for b in self.bodies if b.id == body_id), None)
            if not body_to_delete:
                continue
            
            print(f"  Deleting body {body_id}: {body_to_delete.name}")
            
            # Delete joints connected to this body
            joints_to_delete = [joint_name for joint_name, joint in self.joints.items()
                              if joint.body1_id == body_id or joint.body2_id == body_id]
            
            for joint_name in joints_to_delete:
                print(f"    Removing joint: {joint_name}")
                self.joint_renderer.remove_joint(joint_name)
                del self.joints[joint_name]
            
            # Delete forces on this body
            forces_to_delete = [force_name for force_name, force in self.forces.items()
                               if force.body_id == body_id]
            
            for force_name in forces_to_delete:
                print(f"    Removing force: {force_name}")
                self.force_renderer.remove_force(force_name)
                del self.forces[force_name]
            
            # Delete torques on this body
            torques_to_delete = [torque_name for torque_name, torque in self.torques.items()
                                if torque.body_id == body_id]
            
            for torque_name in torques_to_delete:
                print(f"    Removing torque: {torque_name}")
                self.torque_renderer.remove_torque(torque_name)
                del self.torques[torque_name]
            
            # Delete frames associated with this body
            frames_to_delete = [name for name, bid in self.frame_to_body_map.items() if bid == body_id]
            
            for frame_name in frames_to_delete:
                print(f"    Removing frame: {frame_name}")
                self.frame_renderer.remove_frame(frame_name)
                del self.created_frames[frame_name]
                del self.frame_to_body_map[frame_name]
            
            # Remove body's local frame
            if body_to_delete.local_frame:
                self.frame_renderer.remove_frame(body_to_delete.local_frame.name)
            
            # Remove from renderer
            self.body_renderer.remove_body(body_id)
            
            # Remove from viewer mappings
            self.viewer_3d.remove_body_from_mapping(body_id)
            
            # Remove face, edge, and vertex properties
            if body_id in self.face_properties_map:
                del self.face_properties_map[body_id]
            if body_id in self.edge_properties_map:
                del self.edge_properties_map[body_id]
            if body_id in self.vertex_properties_map:
                del self.vertex_properties_map[body_id]
        
        # Remove all deleted bodies from the bodies list
        self.bodies = [b for b in self.bodies if b.id not in body_ids]
        
        # Clear selection if any deleted body was selected
        if self.selected_body_id in body_ids:
            self.selected_body_id = None
            self.property_panel.show_no_selection()
        
        # Update GUI
        self.body_tree.update_bodies(self.bodies)
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.update_joints_list(self.joints.values())
        self.body_tree.update_forces_list(self.forces.values())
        self.body_tree.update_torques_list(self.torques.values())
        
        # Update display
        self.display.Repaint()
        
        print(f"Successfully deleted {len(body_ids)} bodies")
            
    def on_frame_position_changed(self, frame_name: str, position: Tuple[float, float, float]):
        """Handle manual frame position update"""
        if frame_name in self.created_frames:
            print(f"Updating frame {frame_name} position to {position}")
            frame = self.created_frames[frame_name]
            # Update frame object
            frame.origin = np.array(position)
            # Re-render in viewer
            self.frame_renderer.render_frame(frame, visible=True)

    def on_frame_rotation_changed(self, frame_name: str, angles: Tuple[float, float, float]):
        """Handle manual frame rotation update"""
        if frame_name in self.created_frames:
            print(f"Updating frame {frame_name} rotation to {angles}")
            frame = self.created_frames[frame_name]
            # Update frame object
            frame.set_rotation_from_euler(np.array(angles))
            # Re-render in viewer
            self.frame_renderer.render_frame(frame, visible=True)
            # Update axis display in property panel (avoids full reload to keep focus, 
            # but show_frame_properties blocks signals so it is safe)
            self.property_panel.show_frame_properties(frame)

    def on_body_visibility_changed(self, body_id: int, visible: bool):
        """Handle body visibility change"""
        print(f"Body {body_id} visibility changed to {visible}")
        self.body_renderer.set_body_visibility(body_id, visible)
        
        # Update body object
        for body in self.bodies:
            if body.id == body_id:
                body.visible = visible
                break
    
    def on_contact_detection_changed(self, body_id: int, contact_enabled: bool):
        """Handle contact detection toggle"""
        print(f"Body {body_id} contact detection changed to {contact_enabled}")
        
        # Update body object
        for body in self.bodies:
            if body.id == body_id:
                body.contact_enabled = contact_enabled
                break
    
    def on_isolate_body(self, body_id: int):
        """
        Isolate a specific body - hide all others
        
        Args:
            body_id: ID of the body to isolate
        """
        print(f"Isolating body {body_id}")
        
        # Hide all bodies except the target
        for body in self.bodies:
            visible = (body.id == body_id)
            self.body_renderer.set_body_visibility(body.id, visible)
        
        # Update isolation state
        self.isolation_active = True
        self.isolated_body_id = body_id
        
        QMessageBox.information(
            self, 
            "Isolation Active", 
            f"Body '{next((b.name for b in self.bodies if b.id == body_id), body_id)}' isolated.\nRight-click and select 'Exit Isolation' to show all bodies."
        )
    
    def on_exit_isolation(self):
        """
        Exit isolation mode - show all bodies
        """
        print("Exiting isolation mode")
        
        # Show all bodies
        for body in self.bodies:
            self.body_renderer.set_body_visibility(body.id, True)
        
        # Reset isolation state
        self.isolation_active = False
        self.isolated_body_id = None
        
        print("All bodies visible")

    def create_joint(self):
        """Open dialog to create a new joint"""
        # Ensure we have bodies logic: if only Ground is there, that's not enough usually, but maybe it is for debugging.
        # But ground is always created on init. Step file loading bodies.
        
        all_bodies = [self.ground_body] + self.bodies

        if len(all_bodies) < 2:
             QMessageBox.warning(self, "Not Enough Bodies", "Need at least two bodies (including Ground) to create a joint.")
             return

        # Gather all available frames
        available_frames = [self.world_frame]
        for body in self.bodies: # Don't re-add Ground's frame if it is world frame
            if body.local_frame:
                available_frames.append(body.local_frame)
        available_frames.extend(self.created_frames.values())

        # Open Dialog
        dialog = JointCreationDialog(all_bodies, available_frames, self)
        if dialog.exec():
            name, j_type, b1_id, b2_id, frame, axis = dialog.get_data()
            
            # Validation
            if not name:
                QMessageBox.warning(self, "Invalid Name", "Joint name cannot be empty.")
                return
            if name in self.joints:
                QMessageBox.warning(self, "Duplicate Name", f"Joint '{name}' already exists.")
                return
            if b1_id == b2_id:
                QMessageBox.warning(self, "Invalid Bodies", "Cannot create joint between the same body.")
                return

            # Create Joint (with single frame)
            joint = Joint(name, j_type, b1_id, b2_id, frame, axis)
            self.joints[name] = joint
            
            print(f"Created joint: {joint}")
            
            # Update Tree
            self.body_tree.update_joints_list(self.joints.values())
            
            # Render Joint
            self.joint_renderer.render_joint(joint)
            
            # TODO: Add logic to store joint in an Assembly object if we had one centrally
            
    def on_joint_selected(self, joint_name: str):
        """Handle joint selection in tree"""
        if joint_name in self.joints:
            print(f"Selected Joint: {joint_name}")
            joint = self.joints[joint_name]
            self.property_panel.show_joint_properties(joint)
            
            # Ensure joint is rendered (it should be already)
            # Maybe highlight it? JointRenderer doesn't support highlighting yet explicitly
            # beyond the normal rendering.

    def on_joint_deleted(self, joint_name: str):
        """Handle joint deletion request"""
        if joint_name in self.joints:
            print(f"Deleting joint: {joint_name}")
            
            # Confirm deletion
            reply = QMessageBox.question(
                self, 
                "Confirm Deletion", 
                f"Are you sure you want to delete joint '{joint_name}'?",
                QMessageBox.Yes | QMessageBox.No, 
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Remove visualization
                self.joint_renderer.remove_joint(joint_name)
                
                # Remove from storage
                del self.joints[joint_name]
                
                # Update tree
                self.body_tree.update_joints_list(self.joints.values())
                
                # Clear property panel if this joint was selected
                # We can check by seeing if property panel is showing a joint with this name,
                # but show_no_selection is safer/easier
                self.property_panel.show_no_selection()
                
                print(f"Joint '{joint_name}' deleted.")
    
    def add_motor_to_joint(self):
        """Open dialog to add a motor to an existing joint"""
        if not self.joints:
            QMessageBox.warning(self, "No Joints", "Please create a joint first.")
            return
        
        # Get currently selected joint (if any) from tree or elsewhere
        selected_joint_name = None
        # We could track this more explicitly; for now let user select in dialog
        
        # Open motor dialog
        dialog = MotorDialog(list(self.joints.values()), selected_joint_name, self)
        
        if dialog.exec() == QDialog.Accepted:
            joint, motor_type, value = dialog.get_motor_data()
            
            if not joint:
                return
            
            try:
                # Add motor to joint
                joint.add_motor(motor_type, value)
                
                print(f"Added {motor_type.name} motor to joint '{joint.name}' with value {value}")
                
                # Update tree to show motor indicator
                self.body_tree.update_joints_list(self.joints.values())
                
                # Update visualization
                self.motor_renderer.render_motor(joint, self.bodies, self.ground_body)
                
                # Refresh property panel if this joint is selected
                self.property_panel.show_joint_properties(joint)
                
                QMessageBox.information(self, "Motor Added", 
                                      f"Motor added to joint '{joint.name}' successfully!")
                
            except ValueError as e:
                QMessageBox.critical(self, "Error", f"Failed to add motor:\n{str(e)}")
    
    def remove_motor_from_joint(self):
        """Remove motor from a joint"""
        if not self.joints:
            QMessageBox.warning(self, "No Joints", "No joints available.")
            return
        
        # Find motorized joints
        motorized_joints = [j for j in self.joints.values() if j.is_motorized]
        
        if not motorized_joints:
            QMessageBox.information(self, "No Motors", "No motorized joints found.")
            return
        
        # Simple selection dialog - use a combo box in a message box
        from PySide6.QtWidgets import QInputDialog
        
        joint_names = [j.name for j in motorized_joints]
        joint_name, ok = QInputDialog.getItem(
            self, 
            "Remove Motor", 
            "Select joint to remove motor from:",
            joint_names,
            0,
            False
        )
        
        if ok and joint_name:
            joint = self.joints[joint_name]
            
            # Confirm removal
            reply = QMessageBox.question(
                self,
                "Confirm Removal",
                f"Remove motor from joint '{joint_name}'?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Remove motor
                joint.remove_motor()
                
                # Remove visualization
                self.motor_renderer.remove_motor(joint_name)
                
                # Update tree
                self.body_tree.update_joints_list(self.joints.values())
                
                # Refresh property panel
                self.property_panel.show_joint_properties(joint)
                
                print(f"Motor removed from joint '{joint_name}'")
                QMessageBox.information(self, "Motor Removed", 
                                      f"Motor removed from joint '{joint_name}'.")
    
    def add_force(self):
        """Open dialog to create a new force"""
        if not self.bodies:
            QMessageBox.warning(self, "No Bodies", "Please load a STEP file first.")
            return
        
        # Gather all bodies including ground
        all_bodies = [self.ground_body] + self.bodies
        
        # Gather all available frames
        available_frames = [self.world_frame]
        for body in self.bodies:
            if body.local_frame:
                available_frames.append(body.local_frame)
        available_frames.extend(self.created_frames.values())
        
        # Open Dialog (pre-select current body if any)
        dialog = ForceDialog(all_bodies, available_frames, self.selected_body_id, self)
        
        if dialog.exec() == QDialog.Accepted:
            name, body_id, frame, magnitude, direction = dialog.get_data()
            
            # Validation
            if not name:
                QMessageBox.warning(self, "Invalid Name", "Force name cannot be empty.")
                return
            if name in self.forces:
                QMessageBox.warning(self, "Duplicate Name", f"Force '{name}' already exists.")
                return
            if magnitude <= 0:
                QMessageBox.warning(self, "Invalid Magnitude", "Force magnitude must be positive.")
                return
            
            # Create Force
            try:
                force = Force(name, body_id, frame, magnitude, direction)
                self.forces[name] = force
                
                print(f"Created force: {force}")
                
                # Update Tree
                self.body_tree.update_forces_list(self.forces.values())
                
                # Render Force
                self.force_renderer.render_force(force)
                
                QMessageBox.information(self, "Force Created", f"Force '{name}' created successfully!")
                
            except ValueError as e:
                QMessageBox.critical(self, "Error", f"Failed to create force:\n{str(e)}")
    
    def on_force_selected(self, force_name: str):
        """Handle force selection in tree"""
        if force_name in self.forces:
            print(f"Selected Force: {force_name}")
            force = self.forces[force_name]
            self.property_panel.show_force_properties(force)
    
    def on_force_deleted(self, force_name: str):
        """Handle force deletion request"""
        if force_name in self.forces:
            print(f"Deleting force: {force_name}")
            
            # Confirm deletion
            reply = QMessageBox.question(
                self, 
                "Confirm Deletion", 
                f"Are you sure you want to delete force '{force_name}'?",
                QMessageBox.Yes | QMessageBox.No, 
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Remove visualization
                self.force_renderer.remove_force(force_name)
                
                # Remove from storage
                del self.forces[force_name]
                
                # Update tree
                self.body_tree.update_forces_list(self.forces.values())
                
                # Clear property panel
                self.property_panel.show_no_selection()
                
                print(f"Force '{force_name}' deleted.")

    def add_torque(self):
        """Open dialog to create a new torque"""
        if not self.bodies:
            QMessageBox.warning(self, "No Bodies", "Please load a STEP file first.")
            return
        
        # Gather all bodies including ground
        all_bodies = [self.ground_body] + self.bodies
        
        # Gather all available frames
        available_frames = [self.world_frame]
        for body in self.bodies:
            if body.local_frame:
                available_frames.append(body.local_frame)
        available_frames.extend(self.created_frames.values())
        
        # Open Dialog (pre-select current body if any)
        dialog = TorqueDialog(all_bodies, available_frames, self.selected_body_id, self)
        
        if dialog.exec() == QDialog.Accepted:
            name, body_id, frame, magnitude, axis = dialog.get_data()
            
            # Validation
            if not name:
                QMessageBox.warning(self, "Invalid Name", "Torque name cannot be empty.")
                return
            if name in self.torques:
                QMessageBox.warning(self, "Duplicate Name", f"Torque '{name}' already exists.")
                return
            if magnitude <= 0:
                QMessageBox.warning(self, "Invalid Magnitude", "Torque magnitude must be positive.")
                return
            
            # Create Torque
            try:
                torque = Torque(name, body_id, frame, magnitude, axis)
                self.torques[name] = torque
                
                print(f"Created torque: {torque}")
                
                # Update Tree
                self.body_tree.update_torques_list(self.torques.values())
                
                # Render Torque
                self.torque_renderer.render_torque(torque)
                
                QMessageBox.information(self, "Torque Created", f"Torque '{name}' created successfully!")
                
            except ValueError as e:
                QMessageBox.critical(self, "Error", f"Failed to create torque:\n{str(e)}")
    
    def on_torque_selected(self, torque_name: str):
        """Handle torque selection in tree"""
        if torque_name in self.torques:
            print(f"Selected Torque: {torque_name}")
            torque = self.torques[torque_name]
            self.property_panel.show_torque_properties(torque)
    
    def on_torque_deleted(self, torque_name: str):
        """Handle torque deletion request"""
        if torque_name in self.torques:
            print(f"Deleting torque: {torque_name}")
            
            # Confirm deletion
            reply = QMessageBox.question(
                self, 
                "Confirm Deletion", 
                f"Are you sure you want to delete torque '{torque_name}'?",
                QMessageBox.Yes | QMessageBox.No, 
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Remove visualization
                self.torque_renderer.remove_torque(torque_name)
                
                # Remove from storage
                del self.torques[torque_name]
                
                # Update tree
                self.body_tree.update_torques_list(self.torques.values())
                
                # Clear property panel
                self.property_panel.show_no_selection()
                
                print(f"Torque '{torque_name}' deleted.")

    def create_test_joint(self):
        """Create and visualize a test joint"""

        if not self.bodies:
            QMessageBox.warning(self, "No Bodies", "Please load a STEP file first.")
            return

        print("Creating test joint...")
        
        # Pick two bodies (or just use Body 0 and 1 if available, else Body 0 and World)
        body1_id = self.bodies[0].id
        body2_id = self.bodies[1].id if len(self.bodies) > 1 else -1 
        
        # Define arbitrary frames
        # Frame 1: at Body 1 COM + offset
        f1_origin = self.bodies[0].center_of_mass if self.bodies[0].center_of_mass is not None else [0,0,0]
        
        # Calculate a reasonable offset based on model size (use unit scaling if available, else guess)
        # Using 5% of max bound or 5cm (scaled) as fallback
        offset_val = 0.05  # Default 5cm
        if hasattr(self, 'unit_scale'):
             # If units are small (e.g. m), 0.05 is 5cm.
             # If units are mm, 0.05 is 0.05 meters (50mm).
             # Wait, our physics calc returns meters. So 0.05 is always 5cm.
             pass

        # Offset slightly by 5cm (scaled to meters).
        f1_origin = np.array(f1_origin) + np.array([offset_val, offset_val, offset_val])
        
        f1 = Frame(origin=f1_origin, name="TestJoint_Frame")
        
        joint = Joint("TestJoint", JointType.REVOLUTE, body1_id, body2_id, f1)
        
        self.joint_renderer.render_joint(joint)
        print(f"Test joint rendered at {f1_origin}")
        
        QMessageBox.information(self, "Test Joint", "Created 'TestJoint' with frame at body COM.\nLook for joint frame in viewer.")

    def export_assembly_json(self):
        """Export the complete assembly to JSON format"""
        if not self.bodies:
            QMessageBox.warning(
                self,
                "No Data",
                "No assembly loaded. Please load a STEP file first."
            )
            return
        
        # Ask user for save location
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Export Assembly as JSON",
            "assembly.json",
            "JSON Files (*.json);;All Files (*.*)"
        )
        
        if filepath:
            # Perform export
            success = AssemblyExporter.export_assembly_to_json(
                bodies=self.bodies,
                joints=self.joints,
                frames=self.created_frames,
                ground_body=self.ground_body,
                unit_scale=self.unit_scale,
                output_path=filepath
            )
            
            if success:
                QMessageBox.information(
                    self,
                    "Export Complete",
                    f"Assembly and meshes exported successfully.\n"
                    f"JSON: {filepath}\n"
                    f"Meshes: ./meshes/\n\n"
                    f"Bodies: {len(self.bodies)}\n"
                    f"Joints: {len(self.joints)}\n"
                    f"Frames: {len(self.created_frames)}"
                )
            else:
                QMessageBox.critical(
                    self,
                    "Export Failed",
                    "Failed to export assembly. Check console for details."
                )
    
    def export_body_meshes_obj(self):
        """Export all body geometries as OBJ mesh files"""
        if not self.bodies:
            QMessageBox.warning(
                self,
                "No Data",
                "No assembly loaded. Please load a STEP file first."
            )
            return
        
        # Ask user for output directory
        output_dir = QFileDialog.getExistingDirectory(
            self,
            "Select Directory for OBJ Export",
            "",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
        )
        
        if output_dir:
            # Perform export
            success = AssemblyExporter.export_body_meshes_to_obj(
                bodies=self.bodies,
                output_dir=output_dir,
                unit_scale=self.unit_scale
            )
            
            if success:
                QMessageBox.information(
                    self,
                    "Export Complete",
                    f"Body meshes exported successfully to:\n{output_dir}\n\n"
                    f"Exported {len(self.bodies)} body mesh(es) as OBJ files."
                )
            else:
                QMessageBox.critical(
                    self,
                    "Export Failed",
                    "Failed to export body meshes. Check console for details."
                )
    
    def save_project(self):
        """Save the current project (STEP file path, frames, joints) to a .mbdp file"""
        if not self.current_step_file:
            QMessageBox.warning(self, "No STEP File", "Please load a STEP file before saving a project.")
            return
        
        # Ask user for save location
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Save Project",
            "",
            "MBD Project Files (*.mbdp);;JSON Files (*.json);;All Files (*.*)"
        )
        
        if not filepath:
            print("Project save cancelled by user")
            return
        
        try:
            import json
            import os
            
            # Prepare project data
            project_data = {
                "version": "1.0",
                "step_file": os.path.abspath(self.current_step_file),
                "unit_scale": self.unit_scale,
                "frames": [],
                "joints": []
            }
            
            # Serialize created frames
            for frame_name, frame in self.created_frames.items():
                frame_data = {
                    "name": frame.name,
                    "origin": frame.origin.tolist(),
                    "rotation_matrix": frame.rotation_matrix.tolist()
                }
                project_data["frames"].append(frame_data)
            
            # Serialize joints
            for joint_name, joint in self.joints.items():
                joint_data = {
                    "name": joint.name,
                    "type": joint.joint_type.name,
                    "body1_id": joint.body1_id,
                    "body2_id": joint.body2_id,
                    "frame_name": joint.frame.name,
                    "frame_origin": joint.frame.origin.tolist(),
                    "frame_rotation": joint.frame.rotation_matrix.tolist(),
                    "axis": joint.axis
                }
                project_data["joints"].append(joint_data)
            
            # Write to file
            with open(filepath, 'w') as f:
                json.dump(project_data, f, indent=2)
            
            print(f"Project saved to: {filepath}")
            print(f"  - {len(project_data['frames'])} frames")
            print(f"  - {len(project_data['joints'])} joints")
            
            QMessageBox.information(
                self,
                "Project Saved",
                f"Project saved successfully!\n\nFrames: {len(project_data['frames'])}\nJoints: {len(project_data['joints'])}"
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save project:\n{str(e)}")
            print(f"Error saving project: {e}")
    
    def load_project(self):
        """Load a project from a .mbdp file"""
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "Load Project",
            "",
            "MBD Project Files (*.mbdp);;JSON Files (*.json);;All Files (*.*)"
        )
        
        if not filepath:
            print("Project load cancelled by user")
            return
        
        try:
            import json
            import os
            
            # Read project file
            with open(filepath, 'r') as f:
                project_data = json.load(f)
            
            print(f"Loading project from: {filepath}")
            
            # Verify version
            if project_data.get("version") != "1.0":
                QMessageBox.warning(self, "Version Mismatch", "This project was created with a different version.")
            
            # Load the STEP file first
            step_file = project_data.get("step_file")
            if not step_file or not os.path.exists(step_file):
                # Try relative path
                project_dir = os.path.dirname(filepath)
                step_filename = os.path.basename(step_file) if step_file else ""
                step_file = os.path.join(project_dir, step_filename)
                
                if not os.path.exists(step_file):
                    QMessageBox.critical(
                        self,
                        "STEP File Not Found",
                        f"Cannot find STEP file:\n{project_data.get('step_file')}\n\nPlease locate it manually."
                    )
                    # Ask user to locate the STEP file
                    step_file, _ = QFileDialog.getOpenFileName(
                        self,
                        "Locate STEP File",
                        project_dir,
                        "STEP Files (*.step *.stp);;All Files (*.*)"
                    )
                    if not step_file:
                        return
            
            # Load the STEP file
            self.load_step_file(step_file)
            
            # Restore frames
            frames_data = project_data.get("frames", [])
            for frame_data in frames_data:
                frame = Frame(
                    name=frame_data["name"],
                    origin=np.array(frame_data["origin"]),
                    rotation_matrix=np.array(frame_data["rotation_matrix"])
                )
                self.created_frames[frame.name] = frame
                self.frame_renderer.render_frame(frame, visible=True)
            
            # Update frame tree
            self.body_tree.update_frames(self.created_frames.values())
            
            # Restore joints
            joints_data = project_data.get("joints", [])
            for joint_data in joints_data:
                # Recreate frame for the joint
                frame = Frame(
                    name=joint_data["frame_name"],
                    origin=np.array(joint_data["frame_origin"]),
                    rotation_matrix=np.array(joint_data["frame_rotation"])
                )
                
                # Recreate joint
                joint = Joint(
                    name=joint_data["name"],
                    joint_type=JointType[joint_data["type"]],
                    body1_id=joint_data["body1_id"],
                    body2_id=joint_data["body2_id"],
                    frame=frame,
                    axis=joint_data.get("axis", "+Z")
                )
                
                self.joints[joint.name] = joint
                self.joint_renderer.render_joint(joint)
            
            # Update joints tree
            self.body_tree.update_joints_list(self.joints.values())
            
            print(f"Project loaded successfully!")
            print(f"  - {len(frames_data)} frames restored")
            print(f"  - {len(joints_data)} joints restored")
            
            QMessageBox.information(
                self,
                "Project Loaded",
                f"Project loaded successfully!\n\nFrames: {len(frames_data)}\nJoints: {len(joints_data)}"
            )
            
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "Invalid Project File", f"Failed to parse project file:\n{str(e)}")
            print(f"Error parsing project file: {e}")
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to load project:\n{str(e)}")
            print(f"Error loading project: {e}")
            import traceback
            traceback.print_exc()


def main():
    """Application entry point"""
    app = QApplication(sys.argv)
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Start the event loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()