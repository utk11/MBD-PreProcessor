

import os
import sys
from pathlib import Path


def _ensure_pyside6_qt_plugins() -> None:
    """Prefer pip PySide6 platform plugins over conda qt6-main (version mismatch).

    Conda may ship qt6-main 6.11 while pip PySide6 bundles Qt 6.10. With an empty
    QT_PLUGIN_PATH, Qt can pick $CONDA_PREFIX/lib/qt6/plugins, reject those .so
    files as incompatible, and fail with: Could not find the Qt platform plugin \"xcb\".
    """
    try:
        import PySide6
    except ImportError:
        return
    pyside_file = getattr(PySide6, "__file__", None)
    if not pyside_file:
        return
    plugins = Path(pyside_file).resolve().parent / "Qt" / "plugins"
    if not plugins.is_dir():
        return
    plugins_s = str(plugins)
    # Prepend so PySide6 wins over any conda qt6 plugin dirs already on the path.
    existing = [p for p in os.environ.get("QT_PLUGIN_PATH", "").split(os.pathsep) if p]
    ordered = [plugins_s] + [p for p in existing if p != plugins_s]
    os.environ["QT_PLUGIN_PATH"] = os.pathsep.join(ordered)
    platforms = plugins / "platforms"
    if platforms.is_dir():
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(platforms)


_ensure_pyside6_qt_plugins()

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                QFileDialog, QMenuBar, QMessageBox, QSplitter, QDialog, QLabel)
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QAction
from typing import List, Optional, Dict, Tuple
import numpy as np

# PythonOCC imports for 3D visualization
from OCC.Display.backend import load_backend
load_backend('pyside6')



# Import core modules
from core.step_parser import StepParser
from core.data_structures import RigidBody, Frame, Joint, JointType, Force, Torque, MotorType, State, Pose
from core.assembly_document import AssemblyDocument
from core.joint_factory import AXIAL_JOINTS, JointFrameOption, axis_alignment, make_joint
from core.transforms import attachment_reference_frame, attachment_world_frame, body_world_frame
from core.physics_calculator import PhysicsCalculator
from core.geometry_utils import GeometryUtils, FaceProperties, EdgeProperties, VertexProperties
from core.project_store import (
    ProjectValidationError,
    prepare_import_document,
    read_project,
    resolve_step_file,
    save_project as write_project_file,
)

# Import GUI modules
from gui.body_tree_widget import BodyTreeWidget
from gui.property_panel import PropertyPanel
from gui.viewer_3d import SelectableViewer3d
from gui.joint_dialog import JointCreationDialog
from gui.force_dialog import ForceDialog
from gui.torque_dialog import TorqueDialog
from gui.motor_dialog import MotorDialog
from gui.application_controller import ApplicationController
from gui.solver_selector import SolverSelector

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

# Kinematic assembly solver (JAX-backed session). The GUI commits results itself.


class ImportPayload:
    """Geometry produced by the import worker. Viewer objects are created later on the GUI thread."""

    def __init__(self, generation: int, filepath: str, bodies: list, unit_scale: float,
                 faces: dict, edges: dict, vertices: dict):
        self.generation = generation
        self.filepath = filepath
        self.bodies = bodies
        self.unit_scale = unit_scale
        self.faces = faces
        self.edges = edges
        self.vertices = vertices


class StepLoadWorker(QThread):
    """Background worker for STEP import, one mass-property pass, and feature extraction.

    The worker owns the shape until it emits a payload. The GUI thread creates
    viewer objects after that. An older payload carries its generation and is
    ignored if a newer import has started.
    """
    progress = Signal(str)
    result = Signal(object)
    error = Signal(int, str)

    def __init__(self, filepath: str, generation: int):
        super().__init__()
        self.filepath = filepath
        self.generation = int(generation)

    def run(self):
        try:
            self.progress.emit("Reading STEP file...")
            shape, unit_scale = StepParser.load_step_file(self.filepath)

            if shape is None:
                self.error.emit(self.generation, "Could not load STEP file (invalid or empty).")
                return

            self.progress.emit("Extracting individual bodies...")
            bodies = StepParser.extract_bodies_from_compound(shape)

            self.progress.emit("Calculating mass properties...")
            PhysicsCalculator.calculate_mass_properties_for_bodies(bodies, unit_scale)

            self.progress.emit("Extracting faces, edges, and vertices...")
            faces = {}
            edges = {}
            vertices = {}
            for body in bodies:
                if body.shape is None:
                    continue
                faces[body.id] = GeometryUtils.extract_faces(body.shape, unit_scale)
                edges[body.id] = GeometryUtils.extract_edges(body.shape, unit_scale)
                vertices[body.id] = GeometryUtils.extract_vertices(body.shape, unit_scale)

            self.result.emit(ImportPayload(
                self.generation, self.filepath, bodies, unit_scale, faces, edges, vertices
            ))

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.error.emit(self.generation, f"Error loading file: {str(e)}")


class MainWindow(QMainWindow):
    """Main application window with 3D viewer"""
    load_finished = Signal(int, bool, str)

    @property
    def assembly_state(self):
        return self.document.state

    @assembly_state.setter
    def assembly_state(self, value):
        self.document.state = value

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
        
        # The document owns bodies, joints, frames, loads, poses, and revisions.
        # The attributes below are the same objects, so existing widget code can
        # keep reading them. Mutations that change connectivity go through the document.
        self.document = AssemblyDocument()
        self.bodies = self.document.bodies
        self.assembly_state = None
        self._load_generation = 0
        self._pending_project = None
        self._load_workers = set()
        self._close_requested = False
        self.control_bridge = None
        self._pending_load_interactive = True

        # --- Drag update throttling for smoothness ---
        # MouseMove can fire at very high rates. We use a timer to apply visual
        # updates at a fixed rate (e.g. 60 FPS) for much higher perceived smoothness.
        self._drag_update_timer = QTimer(self)
        self._drag_update_timer.setInterval(16)  # ~60 FPS target
        self._drag_update_timer.timeout.connect(self._apply_pending_drag_update)
        self._pending_drag_body_id: Optional[int] = None
        self._pending_drag_pos: Optional[np.ndarray] = None
        self._dragging_body_ref: Optional[RigidBody] = None  # cache for speed
        self._last_applied_drag_pos: Optional[np.ndarray] = None  # for drag smoothness epsilon check
        
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

        # Connect body dragging (translates via State + renderer)
        self.viewer_3d.on_body_drag_start = self.on_body_drag_start
        self.viewer_3d.on_body_drag_move = self.on_body_drag_move
        self.viewer_3d.on_body_drag_end = self.on_body_drag_end
        
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

        # Same dictionaries the document owns.
        self.created_frames = self.document.frames
        self.frame_to_body_map = self.document.frame_to_body
        self.joints = self.document.joints
        self.forces = self.document.forces
        self.torques = self.document.torques
        
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

        self.controller = ApplicationController(self)
        self.controller.attach_viewer()
        self.controller.scheduler.shutdown_finished.connect(self._finish_close_if_ready)
        self.create_solver_toolbar()

        print("Application initialized successfully!")
        print("Viewer ready. Use File > Open to load a STEP file.")
        print("Mouse controls: Left-click/drag on body to select & move (in Body mode); Right-drag to rotate view; Middle-drag to pan; Wheel to zoom.")
        print("View menu: snap to Top/Bottom/Front/Back/Left/Right or Isometric views.")

    def closeEvent(self, event):
        """Keep Qt worker owners alive until active work and cleanup finish."""
        if not self._close_requested:
            self._close_requested = True
            if getattr(self, "control_bridge", None) is not None:
                self.control_bridge.close()
            self._drag_update_timer.stop()
            self.setEnabled(False)
            self.controller.shutdown()
        if not self.controller.scheduler.is_stopped or any(worker.isRunning() for worker in self._load_workers):
            event.ignore()
            self.statusBar().showMessage("Closing after the current calculation finishes...")
            return
        super().closeEvent(event)

    def _finish_close_if_ready(self):
        if self._close_requested and self.controller.scheduler.is_stopped and not any(
            worker.isRunning() for worker in self._load_workers
        ):
            self.close()
    
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

        solve_assembly_action = QAction("Solve Assembly", self)
        solve_assembly_action.setShortcut("Ctrl+K")
        solve_assembly_action.triggered.connect(self.solve_assembly)
        assembly_menu.addAction(solve_assembly_action)
        
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

        # View menu
        view_menu = menubar.addMenu("View")
        
        # Use full QAction for clarity and future shortcuts
        top_act = QAction("Top (+Z)", self)
        top_act.triggered.connect(self.set_view_top)
        view_menu.addAction(top_act)

        bot_act = QAction("Bottom (-Z)", self)
        bot_act.triggered.connect(self.set_view_bottom)
        view_menu.addAction(bot_act)

        view_menu.addSeparator()

        front_act = QAction("Front (+Y)", self)
        front_act.triggered.connect(self.set_view_front)
        view_menu.addAction(front_act)

        back_act = QAction("Back (-Y)", self)
        back_act.triggered.connect(self.set_view_back)
        view_menu.addAction(back_act)

        view_menu.addSeparator()

        right_act = QAction("Right (+X)", self)
        right_act.triggered.connect(self.set_view_right)
        view_menu.addAction(right_act)

        left_act = QAction("Left (-X)", self)
        left_act.triggered.connect(self.set_view_left)
        view_menu.addAction(left_act)

        view_menu.addSeparator()

        iso_act = QAction("Isometric", self)
        iso_act.triggered.connect(self.set_view_isometric)
        view_menu.addAction(iso_act)

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

    def create_solver_toolbar(self):
        """Keep the active method visible while comparing drag responsiveness."""
        solver_toolbar = self.addToolBar("Solver")
        solver_toolbar.setObjectName("solverToolbar")
        solver_toolbar.setMovable(False)
        solver_toolbar.addWidget(QLabel("  Linear solver: ", solver_toolbar))
        self.solver_selector = SolverSelector(solver_toolbar)
        solver_toolbar.addWidget(self.solver_selector)
        self.solver_selector.currentIndexChanged.connect(
            lambda _index: self.controller.set_linear_solver(self.solver_selector.currentData())
        )

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
        """Load and display a STEP file (heavy work runs in background thread)."""
        print(f"Loading STEP file: {filepath}")
        self._pending_project = None
        self._start_import(filepath)

    def _start_import(self, filepath, project=None, interactive=True):
        """Import CAD on the worker. The open document stays until this succeeds."""
        if self._close_requested:
            return
        self._load_generation += 1
        self._pending_project = project
        self._pending_load_interactive = interactive
        self.property_panel.set_selection_mode("Body")
        self.setEnabled(False)
        if hasattr(self, 'statusBar'):
            self.statusBar().showMessage("Loading STEP file...")

        self.load_worker = StepLoadWorker(filepath, self._load_generation)
        self._load_workers.add(self.load_worker)
        self.load_worker.progress.connect(self.on_load_progress)
        self.load_worker.result.connect(self.on_load_result)
        self.load_worker.error.connect(self.on_load_error)
        self.load_worker.finished.connect(self._on_load_worker_finished)
        self.load_worker.finished.connect(self.load_worker.deleteLater)
        self.load_worker.start()
        return self._load_generation

    def _clear_ui_for_new_load(self):
        """Clear all previous data (called on main thread before starting load)."""
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
        self.frame_to_body_map.clear()
        self.joints.clear()
        self.joint_renderer.clear()
        self.forces.clear()
        self.force_renderer.clear_all()
        self.torques.clear()
        self.torque_renderer.clear_all_torques()
        self.last_face_selection = None
        self.last_edge_selection = None
        self.last_vertex_selection = None

        self.assembly_state = None
        self.selected_body_id = None

        # Cancel drag state
        if hasattr(self, 'viewer_3d') and self.viewer_3d:
            self.viewer_3d._dragging_body_id = None
            self.viewer_3d._drag_start_screen = None
            self.viewer_3d._drag_start_world_pos = None
            self.viewer_3d._last_drag_screen = None
            self.viewer_3d.selected_body_id = None
            self.viewer_3d._left_press_screen = None
            self.viewer_3d._left_press_body_id = None
            self.viewer_3d._left_moved = False
            self.viewer_3d._left_mode = None

        if hasattr(self, '_drag_update_timer'):
            self._drag_update_timer.stop()
        self._pending_drag_body_id = None
        self._pending_drag_pos = None
        self._dragging_body_ref = None



    def on_load_progress(self, message: str):
        if self._close_requested:
            return
        print(message)
        if hasattr(self, 'statusBar'):
            self.statusBar().showMessage(message)

    def on_load_result(self, payload: ImportPayload):
        """Install an import only after it succeeds, and only if it is still current."""
        if self._close_requested or payload.generation != self._load_generation:
            print(f"Ignoring stale import generation {payload.generation}.")
            return
        project = self._pending_project
        try:
            staged, notes, imported_poses = prepare_import_document(
                payload.bodies, payload.unit_scale, payload.filepath, project
            )
        except ProjectValidationError as exc:
            if getattr(self, "_pending_load_interactive", True):
                QMessageBox.critical(self, "Load Error", str(exc))
            if hasattr(self, "load_finished"):
                self.load_finished.emit(payload.generation, False, str(exc))
            print(f"Project restoration failed; current document kept: {exc}")
            return
        print(f"Load complete. Received {len(payload.bodies)} bodies.")
        self._clear_ui_for_new_load()
        self.document.install(staged)
        self.current_step_file = payload.filepath
        unit_scale = payload.unit_scale
        self.unit_scale = unit_scale

        # Update renderers
        self.frame_renderer.set_unit_scale(unit_scale)
        self.joint_renderer.frame_renderer.set_unit_scale(unit_scale)
        self.body_renderer.set_unit_scale(unit_scale)
        self.force_renderer.set_unit_scale(unit_scale)
        self.torque_renderer.set_unit_scale(unit_scale)
        self.vertex_renderer.set_unit_scale(unit_scale)
        self.viewer_3d.set_unit_scale(unit_scale)

        print(f"Found {len(self.bodies)} body/bodies in the assembly:")
        for body in self.bodies:
            print(f"  - {body.name} (ID: {body.id})")

        print(f"State initialized for {len(self.bodies)} bodies.")

        # Saved placement is already restored. CAD geometry still has its
        # imported placement, so supply that baseline explicitly.
        self.body_tree.update_bodies(self.bodies)
        self.body_renderer.display_bodies(self.bodies, base_poses=imported_poses)

        kept = {body.id for body in self.bodies}
        self.face_properties_map = {bid: values for bid, values in payload.faces.items() if bid in kept}
        self.edge_properties_map = {bid: values for bid, values in payload.edges.items() if bid in kept}
        self.vertex_properties_map = {bid: values for bid, values in payload.vertices.items() if bid in kept}
        self._finish_step_load(unit_scale)
        self.controller.on_document_replaced()
        if project is not None:
            self._show_restored_project(notes, notify=self._pending_load_interactive)
        self.load_finished.emit(payload.generation, True, "\n".join(notes))

    def on_load_error(self, generation: int, message: str):
        if self._close_requested or generation != self._load_generation:
            return
        if self._pending_load_interactive:
            QMessageBox.critical(self, "Load Error", message)
        self.load_finished.emit(generation, False, message)
        print("Load error:", message)
        print("The current project was left unchanged.")
        self.setEnabled(True)
        if hasattr(self, 'statusBar'):
            self.statusBar().showMessage("Load failed")

    def _on_load_worker_finished(self):
        """Re-enable the window after the import thread ends.

        Status text is left to the load result. A successful import may already
        be preparing the solver, and a failure has already reported itself.
        """
        worker = self.sender()
        self._load_workers.discard(worker)
        if worker is getattr(self, "load_worker", None):
            self.load_worker = None
        if self._close_requested:
            self._finish_close_if_ready()
        elif not self._load_workers:
            self.setEnabled(True)

    def _finish_step_load(self, unit_scale: float):
        """Second part of STEP loading that must happen on main thread after bodies exist."""
        bodies_dict = {}
        for body in self.bodies:
            bodies_dict[body.id] = body
            print(f"Body {body.id}: {len(self.face_properties_map.get(body.id, []))} faces, "
                  f"{len(self.edge_properties_map.get(body.id, []))} edges, "
                  f"{len(self.vertex_properties_map.get(body.id, []))} vertices")

        self.viewer_3d.set_body_mapping(self.body_renderer.body_ais_shapes, bodies_dict)

        self.display.FitAll()

        # Scale world frame axes etc. (same as before)
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

            axis_scale = max_dimension * 0.2
            self.frame_renderer.set_axis_scale(axis_scale)
            
            force_scale = max_dimension * 0.3
            self.force_renderer.set_force_scale(force_scale)
            
            torque_scale = max_dimension * 0.2
            self.torque_renderer.set_torque_scale(torque_scale)

            current_visibility = self.frame_renderer.frame_visible.get("World Frame", True)
            self.frame_renderer.render_frame(self.world_frame, visible=current_visibility)

            print(f"Frame axes scaled to {axis_scale:.4f}")
            print(f"Force arrows scaled to {force_scale:.4f}") 
            print(f"Torque circles scaled to {torque_scale:.4f}")

        print(f"STEP file loaded successfully! Total bodies: {len(self.bodies)}")
        self.setEnabled(True)
        if hasattr(self, 'statusBar'):
            self.statusBar().showMessage("Ready")

    def _show_restored_project(self, notes: List[str], notify: bool = True):
        """Draw restored frames, joints, and loads, then report migration limits."""
        for frame in self.created_frames.values():
            self._rerender_user_frame(frame.name)
        for joint in self.joints.values():
            self.joint_renderer.render_joint(joint, bodies=self.bodies, ground_body=self.ground_body)
        for force in self.forces.values():
            self.force_renderer.render_force(force)
        for torque in self.torques.values():
            self.torque_renderer.render_torque(torque)
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.update_joints_list(self.joints.values())
        self.body_tree.update_forces_list(self.forces.values())
        self.body_tree.update_torques_list(self.torques.values())
        if not notify:
            return
        if notes:
            QMessageBox.information(
                self,
                "Project Loaded",
                "Project loaded.\n\n" + "\n".join(f"- {note}" for note in notes),
            )
        else:
            QMessageBox.information(self, "Project Loaded", "Project loaded.")

    def on_body_clicked_in_viewer(self, body_id: int):
        """
        Handle body click selection from the 3D viewer.

        Args:
            body_id: ID of the clicked body, or -1 to clear selection (empty click)
        """
        if body_id is None or body_id < 0:
            self.clear_body_selection()
            return
        # Update tree selection to match clicked body
        self.body_tree.select_body(body_id)
        # Tree selection will trigger on_body_selected automatically

    def clear_body_selection(self):
        """Clear the currently selected body so left-drag returns to camera pan."""
        if self.selected_body_id is not None:
            prev_body = next((b for b in self.bodies if b.id == self.selected_body_id), None)
            if prev_body and prev_body.local_frame:
                try:
                    self.frame_renderer.remove_frame(prev_body.local_frame.name)
                except Exception:
                    pass
        self.selected_body_id = None
        self.viewer_3d.selected_body_id = None
        try:
            self.body_renderer.clear_highlight()
        except Exception as e:
            print(f"clear_body_selection highlight cleanup: {e}")
        print("Body selection cleared")

    # ------------------------------------------------------------------
    # Mouse drag support (updates State and re-renders the body)
    # ------------------------------------------------------------------

    def on_body_drag_start(self, body_id: int):
        """Called when the user starts dragging a body in the viewer."""
        print(f"Drag started on body {body_id}")
        self.on_body_selected(body_id)
        self._dragging_body_ref = next((b for b in self.bodies if b.id == body_id), None)
        self._pending_drag_body_id = body_id
        self._pending_drag_pos = None
        self._last_applied_drag_pos = None
        self.controller.on_drag_start(body_id)
        self._drag_update_timer.start()

    def on_body_drag_move(self, body_id: int, new_world_pos: np.ndarray):
        """Record the latest mouse target. The timer submits it to the solver."""
        self._pending_drag_pos = np.array(new_world_pos, dtype=float, copy=True)

    def on_body_drag_end(self, body_id: int):
        """Keep the last mouse target, then ask for a diagnostic on the settled pose."""
        print(f"Drag ended on body {body_id}")
        self._drag_update_timer.stop()
        pending = None if self._pending_drag_pos is None else np.array(self._pending_drag_pos, copy=True)
        self._pending_drag_body_id = None
        self._pending_drag_pos = None
        self._dragging_body_ref = None
        self._last_applied_drag_pos = None
        constrained = body_id in self.document.constrained_body_ids()
        if pending is not None:
            self.controller.submit_drag_target(body_id, pending)
        if constrained:
            self.controller.finish_drag(body_id)
        else:
            self._refresh_settled_visuals()

    def _apply_pending_drag_update(self):
        """Submit the latest mouse point. The worker replaces any older pending target."""
        if self._pending_drag_body_id is None or self._pending_drag_pos is None:
            return
        body_id = self._pending_drag_body_id
        new_pos = np.array(self._pending_drag_pos, dtype=float, copy=True)
        if self._dragging_body_ref is None or self._dragging_body_ref.state is None:
            return
        try:
            if self._last_applied_drag_pos is not None:
                if np.array_equal(new_pos, self._last_applied_drag_pos):
                    return
            self.controller.submit_drag_target(body_id, new_pos)
            self._last_applied_drag_pos = new_pos
        except Exception as e:
            print(f"Error in throttled drag update for {body_id}: {e}")

    def _on_solver_body_changed(self, body_id: int):
        """Follow a committed pose. The reference frame itself is not rewritten."""
        self._sync_highlight_transforms(body_id)
        self._sync_body_attached_frames(body_id, update_viewer=False)
        if self.selected_body_id != body_id:
            return
        body = self.document.body_by_id(body_id)
        if body is None or body.local_frame is None:
            return
        trsf = self._body_ais_local_trsf(body_id)
        if trsf is None:
            return
        try:
            self.frame_renderer.update_frame_local_trsf(
                body.local_frame.name, trsf, update_viewer=False
            )
        except Exception:
            pass

    def _on_solver_settled(self, _report):
        """Refresh joint graphics and the COM marker after a drag release or assembly solve."""
        self._refresh_settled_visuals()

    def _refresh_settled_visuals(self):
        for joint in self.joints.values():
            self.joint_renderer.render_joint(joint, bodies=self.bodies, ground_body=self.ground_body)
        highlighted = self.body_renderer.currently_highlighted_id
        if highlighted is not None and self.body_renderer.com_marker_visible:
            body = self.document.body_by_id(highlighted)
            if body is not None:
                self.body_renderer._update_com_marker(highlighted)
        if self.selected_body_id is not None:
            self._on_solver_body_changed(self.selected_body_id)

    def _sync_highlight_transforms(self, body_id: int):
        """Apply the body's current local transform to any active sub-shape highlight
        for that body (so face/edge/vertex highlights move with dragged body).
        """
        try:
            ais = self.body_renderer.body_ais_shapes.get(body_id)
            if not ais:
                return
            trsf = ais.LocalTransformation()

            if self.last_face_selection and self.last_face_selection[0] == body_id:
                if self.face_renderer.current_ais:
                    self.face_renderer.current_ais.SetLocalTransformation(trsf)
                    self.display.Context.Redisplay(self.face_renderer.current_ais, False)

            if self.last_edge_selection and self.last_edge_selection[0] == body_id:
                if self.edge_renderer.current_ais:
                    self.edge_renderer.current_ais.SetLocalTransformation(trsf)
                    self.display.Context.Redisplay(self.edge_renderer.current_ais, False)

            if self.last_vertex_selection and self.last_vertex_selection[0] == body_id:
                if self.vertex_renderer.current_ais:
                    self.vertex_renderer.current_ais.SetLocalTransformation(trsf)
                    self.display.Context.Redisplay(self.vertex_renderer.current_ais, False)
        except Exception as e:
            # non-fatal
            pass

    # ------------------------------------------------------------------
    # Standard view snapping (perpendicular to axes + isometric)
    # ------------------------------------------------------------------

    def _set_view(self, px: float, py: float, pz: float, ux: float = 0.0, uy: float = 0.0, uz: float = 1.0):
        """Helper to set camera projection and up vector, then fit."""
        v = self.display.View
        v.SetProj(px, py, pz)
        v.SetUp(ux, uy, uz)
        v.FitAll()
        self.display.Context.UpdateCurrentViewer()

    def set_view_top(self):
        """Snap to top view (looking along +Z, Y up)."""
        self._set_view(0.0, 0.0, 1.0, 0.0, 1.0, 0.0)

    def set_view_bottom(self):
        """Snap to bottom view (looking along -Z, Y up)."""
        self._set_view(0.0, 0.0, -1.0, 0.0, 1.0, 0.0)

    def set_view_front(self):
        """Snap to front view (looking along +Y, Z up)."""
        self._set_view(0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

    def set_view_back(self):
        """Snap to back view (looking along -Y, Z up)."""
        self._set_view(0.0, -1.0, 0.0, 0.0, 0.0, 1.0)

    def set_view_right(self):
        """Snap to right view (looking along +X, Z up)."""
        self._set_view(1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def set_view_left(self):
        """Snap to left view (looking along -X, Z up)."""
        self._set_view(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def set_view_isometric(self):
        """Snap to isometric view (approx. 1,1,1 direction, Z up)."""
        self._set_view(1.0, 1.0, 1.0, 0.0, 0.0, 1.0)

    def on_body_selected(self, body_id: int):
        """
        Handle body selection from the tree widget

        Args:
            body_id: ID of the selected body
        """
        print(f"Highlighting body {body_id} in viewer")
        self.body_renderer.highlight_body(body_id)

        # Ensure the property panel and viewer are in Body selection mode (important for dragging)
        self.property_panel.set_selection_mode("Body")
        self.viewer_3d.set_selection_mode("Body")
        
        # Hide previous body's local frame if any
        if self.selected_body_id is not None:
            prev_body = next((b for b in self.bodies if b.id == self.selected_body_id), None)
            if prev_body and prev_body.local_frame:
                self.frame_renderer.remove_frame(prev_body.local_frame.name)
        
        # Update selected body ID
        self.selected_body_id = body_id
        self.viewer_3d.selected_body_id = body_id
        
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
                trsf = self._body_ais_local_trsf(selected_body.id)
                self.frame_renderer.render_frame(
                    selected_body.local_frame, visible=is_visible, local_trsf=trsf
                )

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
            
            # Highlight the face, applying body's current pose transform so it follows drags
            trsf = None
            body_ais = self.body_renderer.body_ais_shapes.get(body_id)
            if body_ais:
                try:
                    trsf = body_ais.LocalTransformation()
                except:
                    pass
            self.face_renderer.highlight_face(face_props.face, trsf)
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
            
            # Highlight the edge, applying body's current pose transform so it follows drags
            trsf = None
            body_ais = self.body_renderer.body_ais_shapes.get(body_id)
            if body_ais:
                try:
                    trsf = body_ais.LocalTransformation()
                except:
                    pass
            self.edge_renderer.highlight_edge(edge_props.edge, trsf)
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
            
            # Highlight the vertex, applying body's current pose transform so it follows drags
            trsf = None
            body_ais = self.body_renderer.body_ais_shapes.get(body_id)
            if body_ais:
                try:
                    trsf = body_ais.LocalTransformation()
                except:
                    pass
            self.vertex_renderer.highlight_vertex(vertex_props.vertex, trsf)
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

        # Origin/orientation in body geometry space (meters). Do NOT bake State
        # pose here — the body's AIS LocalTransformation is applied at render
        # time, matching face highlights exactly.
        frame = GeometryUtils.frame_from_face(face_props, name=frame_name, unit_scale=self.unit_scale)

        self.document.add_frame(frame, body_id, "reference_geometry")
        trsf = self._body_ais_local_trsf(body_id)
        self.frame_renderer.render_frame(frame, visible=True, local_trsf=trsf)

        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)

        self.property_panel.show_frame_properties(frame)
        cm = getattr(face_props, "center_model", None)
        print(
            f"Created frame '{frame_name}' from face {face_index} on body {body_id} "
            f"origin_m={frame.origin} center_model={cm}"
        )

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

        self.document.add_frame(frame, body_id, "reference_geometry")
        trsf = self._body_ais_local_trsf(body_id)
        self.frame_renderer.render_frame(frame, visible=True, local_trsf=trsf)

        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)

        self.property_panel.show_frame_properties(frame)
        print(f"Created frame '{frame_name}' from edge {edge_index} on body {body_id} at {frame.origin}")

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

        self.document.add_frame(frame, body_id, "reference_geometry")
        trsf = self._body_ais_local_trsf(body_id)
        self.frame_renderer.render_frame(frame, visible=True, local_trsf=trsf)

        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.select_frame(frame_name)

        self.property_panel.show_frame_properties(frame)
        print(f"Created frame '{frame_name}' from vertex {vertex_index} on body {body_id} at {frame.origin}")

    def _rerender_user_frame(self, frame_name: str):
        """Draw a user frame in its stored coordinates, following its parent body."""
        frame = self.created_frames.get(frame_name)
        if frame is None:
            return
        body_id = self.frame_to_body_map.get(frame_name)
        coordinates = self.document.frame_coordinates.get(frame_name, "world")
        body = self.ground_body if body_id == -1 else self.document.body_by_id(body_id) if body_id is not None else None
        reference = attachment_reference_frame(frame, body, coordinates)
        trsf = None if body_id is None or coordinates == "world" else self._body_ais_local_trsf(body_id)
        self.frame_renderer.render_frame(reference, visible=True, local_trsf=trsf)

    def _available_world_frames(self):
        """Dialogs consume world coordinates, never mutable reference frames."""
        frames = [Frame(self.world_frame.origin, self.world_frame.rotation_matrix, self.world_frame.name)]
        frames.extend(body_world_frame(body) for body in self.bodies if body.local_frame is not None)
        for name, frame in self.created_frames.items():
            parent = self.frame_to_body_map.get(name)
            body = self.ground_body if parent == -1 else self.document.body_by_id(parent) if parent is not None else None
            frames.append(attachment_world_frame(
                frame, body, self.document.frame_coordinates.get(name, "world")
            ))
        return frames

    def _body_ais_local_trsf(self, body_id: int):
        """Return the body's current AIS LocalTransformation (or None)."""
        try:
            ais = self.body_renderer.body_ais_shapes.get(body_id)
            if ais is None:
                return None
            return ais.LocalTransformation()
        except Exception:
            return None

    def _sync_body_attached_frames(self, body_id: int, update_viewer: bool = False):
        """Keep user frames parented to a body aligned after pose changes."""
        trsf = self._body_ais_local_trsf(body_id)
        for fname, bid in list(self.frame_to_body_map.items()):
            if bid == body_id and fname in self.created_frames:
                if self.document.frame_coordinates.get(fname, "world") == "world":
                    continue
                try:
                    self.frame_renderer.update_frame_local_trsf(
                        fname, trsf, update_viewer=update_viewer
                    )
                except Exception:
                    # Fallback: full re-render
                    try:
                        self._rerender_user_frame(fname)
                    except Exception as e:
                        print(f"Frame sync failed for {fname}: {e}")

    def _body_delta_transform(self, body_id: int):
        """Return (delta_origin_m, delta_rot) mapping original geometry → current world.

        Matches BodyRenderer.update_body_transform. Kept for solvers/export helpers.
        """
        if self.assembly_state is None:
            return np.zeros(3), np.eye(3)
        desired = self.assembly_state.get_body_pose(body_id)
        if desired is None:
            return np.zeros(3), np.eye(3)
        base = self.body_renderer._base_poses.get(body_id)
        if base is None:
            return np.zeros(3), np.eye(3)
        delta_rot = desired.rotation_matrix @ base.rotation_matrix.T
        delta_origin = desired.origin - (delta_rot @ base.origin)
        return delta_origin, delta_rot

    def _apply_body_pose_to_frame(self, frame: Frame, body_id: int):
        """Transform a frame defined in original body geometry space into world space."""
        delta_origin, delta_rot = self._body_delta_transform(body_id)
        frame.origin = delta_rot @ np.asarray(frame.origin, dtype=float) + delta_origin
        frame.rotation_matrix = delta_rot @ np.asarray(frame.rotation_matrix, dtype=float)
    
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
            self.frame_renderer.remove_frame(frame_name)
            self.document.delete_frame(frame_name)
            self.body_tree.update_frames(self.created_frames.values())
            # Clear property panel
            self.property_panel.show_no_selection()
    
    def on_body_deleted(self, body_id: int):
        """Handle body deletion request."""
        self._delete_bodies([body_id])

    def on_multiple_bodies_deleted(self, body_ids: List[int]):
        """Handle multiple body deletion request."""
        self._delete_bodies(list(body_ids))

    def _delete_bodies(self, body_ids: List[int]):
        """Remove bodies and every joint, frame, force, and torque that hangs off them."""
        seen = []
        for body_id in body_ids:
            if body_id not in seen:
                seen.append(body_id)
        bodies = [body for body in self.bodies if body.id in set(seen)]
        if not bodies:
            print("No bodies found to delete")
            return
        if len(bodies) == 1:
            title = "Delete Body"
            message = (
                f"Are you sure you want to delete '{bodies[0].name}'?\n\n"
                "This will also delete:\n"
                "- Associated frames\n"
                "- Joints connected to this body\n"
                "- Forces and torques on this body"
            )
        else:
            names = "\n".join(f"  - {body.name}" for body in bodies)
            title = "Delete Multiple Bodies"
            message = (
                f"Are you sure you want to delete {len(bodies)} bodies?\n\n{names}\n\n"
                "This will also delete:\n"
                "- Associated frames\n"
                "- Joints connected to these bodies\n"
                "- Forces and torques on these bodies"
            )
        reply = QMessageBox.question(
            self, title, message, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        local_frames = [
            (body.id, body.local_frame.name)
            for body in bodies
            if body.local_frame is not None
        ]
        change = self.document.delete_bodies(seen)
        for name in change.deleted_joints:
            self.joint_renderer.remove_joint(name)
        for name in change.deleted_frames:
            self.frame_renderer.remove_frame(name)
        for name in change.deleted_forces:
            self.force_renderer.remove_force(name)
        for name in change.deleted_torques:
            self.torque_renderer.remove_torque(name)
        for body_id, frame_name in local_frames:
            self.frame_renderer.remove_frame(frame_name)
            self.body_renderer.remove_body(body_id)
            self.viewer_3d.remove_body_from_mapping(body_id)
            self.face_properties_map.pop(body_id, None)
            self.edge_properties_map.pop(body_id, None)
            self.vertex_properties_map.pop(body_id, None)
        if self.selected_body_id in set(change.deleted_body_ids):
            self.selected_body_id = None
            self.viewer_3d.selected_body_id = None
            self.property_panel.show_no_selection()
        self.body_tree.update_bodies(self.bodies)
        self.body_tree.update_frames(self.created_frames.values())
        self.body_tree.update_joints_list(self.joints.values())
        self.body_tree.update_forces_list(self.forces.values())
        self.body_tree.update_torques_list(self.torques.values())
        if self.controller.coordinator is not None:
            self.controller.coordinator.forget(change.deleted_body_ids)
        self.controller.on_topology_changed()
        self.display.Repaint()
        print(f"Deleted bodies: {change.deleted_body_ids}")
            
    def on_frame_position_changed(self, frame_name: str, position: Tuple[float, float, float]):
        """Handle manual frame position update"""
        if frame_name in self.created_frames:
            print(f"Updating frame {frame_name} position to {position}")
            frame = self.created_frames[frame_name]
            # Update frame object
            frame.origin = np.array(position)
            self._rerender_user_frame(frame_name)

    def on_frame_rotation_changed(self, frame_name: str, angles: Tuple[float, float, float]):
        """Handle manual frame rotation update"""
        if frame_name in self.created_frames:
            print(f"Updating frame {frame_name} rotation to {angles}")
            frame = self.created_frames[frame_name]
            # Update frame object
            frame.set_rotation_from_euler(np.array(angles))
            self._rerender_user_frame(frame_name)
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
        frame_options = {
            int(body.id): self._joint_frames_for_body(int(body.id))
            for body in all_bodies
        }
        if any(not options for options in frame_options.values()):
            QMessageBox.information(
                self, "Joint Frames Needed",
                "Each body needs an available frame. Create a frame on the intended "
                "face, edge, or vertex, then create the joint.",
            )
            return

        dialog = JointCreationDialog(all_bodies, frame_options, self)
        dialog.preview_changed.connect(self._preview_joint_frames)
        try:
            accepted = dialog.exec()
        finally:
            self.joint_renderer.clear_preview()
        if not accepted:
            return

        request = dialog.get_data()
        name = request["name"]
        if name in self.joints:
            QMessageBox.warning(self, "Duplicate Name", f"Joint '{name}' already exists.")
            return
        bodies_by_id = {-1: self.ground_body, **{int(body.id): body for body in self.bodies}}
        try:
            joint = make_joint(
                name=name,
                joint_type=request["joint_type"],
                body1_id=request["body1_id"],
                body2_id=request["body2_id"],
                frame1=request["frame1"],
                frame2=request["frame2"],
                body1=bodies_by_id.get(int(request["body1_id"])),
                body2=bodies_by_id.get(int(request["body2_id"])),
                pose1=self._body_pose_tuple(int(request["body1_id"])),
                pose2=self._body_pose_tuple(int(request["body2_id"])),
                axis1=request["axis1"],
                axis2=request["axis2"],
                flip1=request["flip1"],
                flip2=request["flip2"],
            )
        except (TypeError, ValueError) as error:
            QMessageBox.warning(self, "Invalid Joint Frames", str(error))
            return

        self.document.add_joint(joint)
        self.controller.on_topology_changed()
        self.body_tree.update_joints_list(self.joints.values())
        self.joint_renderer.render_joint(
            joint, bodies=self.bodies, ground_body=self.ground_body
        )
        print(f"Created joint: {joint}")
        if request["assemble"]:
            self.controller.solve_assembly(
                require_feasible=True,
                reference_body_id=request["reference_body_id"],
            )

    def _joint_frames_for_body(self, body_id: int):
        """List only frames owned by this joint endpoint's body."""
        if int(body_id) == -1:
            options = [JointFrameOption(
                self.world_frame.name, -1,
                Frame(self.world_frame.origin.copy(), self.world_frame.rotation_matrix.copy(), self.world_frame.name),
                "world",
            )]
        else:
            body = self.document.body_by_id(body_id)
            if body is None:
                return []
            options = []
            if body.local_frame is not None:
                options.append(JointFrameOption(
                    body.local_frame.name, body_id, body.local_frame, "reference_geometry"
                ))
        for name, parent_id in self.frame_to_body_map.items():
            if int(parent_id) == int(body_id) and name in self.created_frames:
                options.append(JointFrameOption(
                    name, int(body_id), self.created_frames[name],
                    self.document.frame_coordinates.get(name, "world"),
                ))
        # Tree and labels are name based, so present one option for duplicate aliases.
        unique = {}
        for option in options:
            unique.setdefault(option.key, option)
        return list(unique.values())

    def _preview_joint_frames(self, frame1, frame2):
        """Show both currently selected frames while the joint dialog is open."""
        if frame1 is None or frame2 is None:
            self.joint_renderer.clear_preview()
            return
        body_by_id = {-1: self.ground_body, **{int(body.id): body for body in self.bodies}}
        try:
            world1 = self._resolve_joint_preview_frame(frame1, body_by_id.get(frame1.owner_body_id))
            world2 = self._resolve_joint_preview_frame(frame2, body_by_id.get(frame2.owner_body_id))
            dialog = self.sender()
            if dialog is not None:
                request = dialog.get_data()
                if request["joint_type"] in AXIAL_JOINTS:
                    world1.rotation_matrix = world1.rotation_matrix @ axis_alignment(request["axis1"])
                    world2.rotation_matrix = world2.rotation_matrix @ axis_alignment(request["axis2"])
                flip_rotation = np.diag([1.0, -1.0, -1.0])
                if request["flip1"]:
                    world1.rotation_matrix = world1.rotation_matrix @ flip_rotation
                if request["flip2"]:
                    world2.rotation_matrix = world2.rotation_matrix @ flip_rotation
            self.joint_renderer.preview_attachments(world1, world2)
        except (TypeError, ValueError, AttributeError):
            self.joint_renderer.clear_preview()

    def _resolve_joint_preview_frame(self, option, body):
        from core.joint_factory import resolve_frame
        pose = self._body_pose_tuple(option.owner_body_id)
        return resolve_frame(option, body, ground_pose=pose)
            
    def _body_pose_tuple(self, body_id: int):
        """Return (origin, rotation_matrix) for a body (or ground) in world coords."""
        if body_id == -1:
            if self.ground_body and self.ground_body.local_frame is not None:
                f = self.ground_body.local_frame
                return f.origin.copy(), f.rotation_matrix.copy()
            return np.zeros(3), np.eye(3)

        if self.assembly_state is not None:
            pose = self.assembly_state.get_body_pose(body_id)
            if pose is not None:
                return pose.origin.copy(), pose.rotation_matrix.copy()

        body = next((b for b in self.bodies if b.id == body_id), None)
        if body is not None and body.local_frame is not None:
            return body.local_frame.origin.copy(), body.local_frame.rotation_matrix.copy()
        return np.zeros(3), np.eye(3)

    def solve_assembly(self):
        """Snap body poses so joint constraints are satisfied (Ctrl+K)."""
        self.controller.solve_assembly()
            
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
                self.joint_renderer.remove_joint(joint_name)
                self.document.delete_joint(joint_name)
                self.controller.on_topology_changed()
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
        available_frames = self._available_world_frames()
        
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
                self.document.add_force(force)
                
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
                self.force_renderer.remove_force(force_name)
                self.document.delete_force(force_name)
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
        available_frames = self._available_world_frames()
        
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
                self.document.add_torque(torque)
                
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
                self.torque_renderer.remove_torque(torque_name)
                self.document.delete_torque(torque_name)
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
        """Save poses, markers, attachments, and loads with the CAD identity."""
        if not self.current_step_file:
            QMessageBox.warning(self, "No STEP File", "Please load a STEP file before saving a project.")
            return
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
            write_project_file(filepath, self.document, self.current_step_file)
            print(f"Project saved to: {filepath}")
            QMessageBox.information(
                self,
                "Project Saved",
                "Project saved successfully.\n\n"
                f"Bodies: {len(self.bodies)}\n"
                f"Joints: {len(self.joints)}\n"
                f"Frames: {len(self.created_frames)}"
            )
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save project:\n{str(e)}")
            print(f"Error saving project: {e}")

    def load_project(self):
        """Read a project, then import its CAD. The open document stays if this fails."""
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
            loaded = read_project(filepath)
        except ProjectValidationError as exc:
            QMessageBox.critical(self, "Invalid Project File", str(exc))
            print(f"Error parsing project file: {exc}")
            return
        step_file = resolve_step_file(filepath, loaded)
        if step_file is None:
            QMessageBox.critical(
                self,
                "STEP File Not Found",
                f"Cannot find STEP file:\n{loaded.step_file}\n\nPlease locate it manually."
            )
            project_dir = os.path.dirname(filepath)
            step_file, _ = QFileDialog.getOpenFileName(
                self,
                "Locate STEP File",
                project_dir,
                "STEP Files (*.step *.stp);;All Files (*.*)"
            )
            if not step_file:
                return
            loaded.step_file = step_file
        print(f"Loading project from: {filepath}")
        self._start_import(step_file, loaded)


def _require_jax():
    """Require the shared JAX residual/Jacobian evaluator for all linear methods."""
    try:
        import jax  # noqa: F401
        import jaxlib  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "JAX is required to run MBD-PreProcessor and is not installed.\n"
            "Install it in this environment with: pip install jax==0.6.2\n"
            f"Details: {exc}"
        )


def main():
    """Application entry point"""
    _require_jax()
    import argparse
    parser = argparse.ArgumentParser(description="Multi-Body Dynamics Preprocessor")
    parser.add_argument("--enable-mcp", action="store_true", help="Enable local application control")
    parser.add_argument("--control-socket", default="mbd-preprocessor", help="Local control socket name")
    options, qt_args = parser.parse_known_args()
    app = QApplication([sys.argv[0], *qt_args])
    
    # Create and show main window
    window = MainWindow()
    if options.enable_mcp:
        from gui.control_bridge import ApplicationControlBridge
        try:
            window.control_bridge = ApplicationControlBridge(window, options.control_socket)
        except Exception:
            window.controller.scheduler.shutdown_finished.connect(app.quit)
            window.controller.shutdown()
            if not window.controller.scheduler.is_stopped:
                app.exec()
            raise
    window.show()
    
    # Start the event loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
