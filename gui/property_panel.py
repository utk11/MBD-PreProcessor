"""
Property panel widget for displaying body/joint/frame information
Increment 19: Manual Frame Translation
Increment 27: Vertex Selection
"""

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QGroupBox, 
                                QFormLayout, QFrame, QCheckBox, QTextEdit, QComboBox, QPushButton, QDoubleSpinBox)
from PySide6.QtCore import Qt, Signal
from typing import Optional, Tuple
from core.data_structures import RigidBody, Frame, Joint, JointType, Force
from core.geometry_utils import VertexProperties
import numpy as np


class PropertyPanel(QWidget):
    """Widget for displaying properties of selected bodies, joints, or frames"""
    
    # Signal emitted when COM visibility is toggled
    com_visibility_changed = Signal(bool)
    
    # Signal emitted when world frame visibility is toggled
    world_frame_visibility_changed = Signal(bool)
    
    # Signal emitted when local frame visibility is toggled
    local_frame_visibility_changed = Signal(bool)
    
    # Signal emitted when frame visibility is toggled
    frame_visibility_changed = Signal(str, bool)  # frame_name, visible
    
    # Signal emitted when frame highlight is toggled
    frame_highlight_changed = Signal(str, bool)  # frame_name, highlighted
    
    # Signal emitted when all frames visibility is toggled
    all_frames_visibility_changed = Signal(bool)  # visible
    
    # Signal emitted when selection mode changes
    selection_mode_changed = Signal(str)  # "Body", "Face", "Edge", or "Vertex"

    # Signal emitted when user requests a frame from a selected face
    create_frame_from_face = Signal()
    
    # Signal emitted when user requests a frame from a selected edge
    create_frame_from_edge = Signal()
    
    # Signal emitted when user requests a frame from a selected vertex
    create_frame_from_vertex = Signal()
    
    # Signal emitted when frame position is changed manually
    frame_position_changed = Signal(str, tuple)  # frame_name, (x, y, z)

    # Signal emitted when frame rotation is changed manually
    frame_rotation_changed = Signal(str, tuple)  # frame_name, (rx, ry, rz)
    
    # Signal emitted when body visibility is changed
    body_visibility_changed = Signal(int, bool)  # body_id, visible
    
    # Signal emitted when contact detection is toggled
    contact_detection_changed = Signal(int, bool)  # body_id, contact_enabled

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create main layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Create title label
        self.title_label = QLabel("Properties")
        self.title_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        layout.addWidget(self.title_label)
        
        # Add separator line
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator)
        
        # Create selection mode dropdown
        mode_layout = QFormLayout()
        self.mode_label = QLabel("Selection Mode:")
        self.mode_dropdown = QComboBox()
        self.mode_dropdown.addItems(["Body", "Face", "Edge", "Vertex"])
        self.mode_dropdown.currentTextChanged.connect(self._on_selection_mode_changed)
        mode_layout.addRow(self.mode_label, self.mode_dropdown)
        
        # Add global frames visibility checkbox
        self.all_frames_visible_checkbox = QCheckBox("Show All Frames")
        self.all_frames_visible_checkbox.setChecked(True)
        self.all_frames_visible_checkbox.clicked.connect(self._on_all_frames_visibility_toggled)
        mode_layout.addRow(self.all_frames_visible_checkbox)
        
        mode_group = QGroupBox("Selection")
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # Add separator line
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.HLine)
        separator2.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator2)
        
        # Create group box for body properties
        self.body_group = QGroupBox("Body Information")
        body_layout = QFormLayout()
        
        # Create labels for body properties
        self.name_label = QLabel("—")
        self.id_label = QLabel("—")
        
        # Visibility checkbox
        self.body_visible_checkbox = QCheckBox("Visible")
        self.body_visible_checkbox.setChecked(True)
        self.body_visible_checkbox.clicked.connect(self._on_body_visibility_toggled)
        
        # Contact detection checkbox
        self.body_contact_checkbox = QCheckBox("Enable Contact Detection")
        self.body_contact_checkbox.setChecked(True)
        self.body_contact_checkbox.clicked.connect(self._on_contact_detection_toggled)
        
        self.volume_label = QLabel("—")
        self.com_label = QLabel("—")
        
        # Create text widget for inertia tensor (multi-line display)
        self.inertia_text = QTextEdit()
        self.inertia_text.setReadOnly(True)
        self.inertia_text.setMaximumHeight(80)
        self.inertia_text.setStyleSheet("font-family: monospace; font-size: 9pt;")
        
        body_layout.addRow("Name:", self.name_label)
        body_layout.addRow("ID:", self.id_label)
        body_layout.addRow("Visibility:", self.body_visible_checkbox)
        body_layout.addRow("Contact:", self.body_contact_checkbox)
        body_layout.addRow("Volume:", self.volume_label)
        body_layout.addRow("Center of Mass:", self.com_label)
        body_layout.addRow("Inertia Tensor:", self.inertia_text)
        
        self.body_group.setLayout(body_layout)
        layout.addWidget(self.body_group)
        
        # Create group box for face properties
        self.face_group = QGroupBox("Face Information")
        face_layout = QFormLayout()
        
        self.face_index_label = QLabel("—")
        self.face_area_label = QLabel("—")
        self.face_center_label = QLabel("—")
        self.face_normal_label = QLabel("—")
        self.face_frame_button = QPushButton("Create Frame from Face")
        self.face_frame_button.setEnabled(False)
        self.face_frame_button.clicked.connect(self._on_create_frame_from_face)
        
        face_layout.addRow("Face Index:", self.face_index_label)
        face_layout.addRow("Area:", self.face_area_label)
        face_layout.addRow("Center:", self.face_center_label)
        face_layout.addRow("Normal:", self.face_normal_label)
        face_layout.addRow(self.face_frame_button)
        
        self.face_group.setLayout(face_layout)
        self.face_group.setVisible(False)  # Hidden by default
        layout.addWidget(self.face_group)
        
        # Create group box for edge properties
        self.edge_group = QGroupBox("Edge Information")
        edge_layout = QFormLayout()
        
        self.edge_index_label = QLabel("—")
        self.edge_length_label = QLabel("—")
        self.edge_midpoint_label = QLabel("—")
        self.edge_direction_label = QLabel("—")
        self.edge_frame_button = QPushButton("Create Frame from Edge")
        self.edge_frame_button.setEnabled(False)
        self.edge_frame_button.clicked.connect(self._on_create_frame_from_edge)
        
        edge_layout.addRow("Edge Index:", self.edge_index_label)
        edge_layout.addRow("Length:", self.edge_length_label)
        edge_layout.addRow("Midpoint:", self.edge_midpoint_label)
        edge_layout.addRow("Direction:", self.edge_direction_label)
        edge_layout.addRow(self.edge_frame_button)
        
        self.edge_group.setLayout(edge_layout)
        self.edge_group.setVisible(False)  # Hidden by default
        layout.addWidget(self.edge_group)
        
        # Create group box for vertex properties
        self.vertex_group = QGroupBox("Vertex Information")
        vertex_layout = QFormLayout()
        
        self.vertex_index_label = QLabel("—")
        self.vertex_x_label = QLabel("—")
        self.vertex_y_label = QLabel("—")
        self.vertex_z_label = QLabel("—")
        self.vertex_frame_button = QPushButton("Create Frame from Vertex")
        self.vertex_frame_button.setEnabled(False)
        self.vertex_frame_button.clicked.connect(self._on_create_frame_from_vertex)
        
        vertex_layout.addRow("Vertex Index:", self.vertex_index_label)
        vertex_layout.addRow("X Coordinate:", self.vertex_x_label)
        vertex_layout.addRow("Y Coordinate:", self.vertex_y_label)
        vertex_layout.addRow("Z Coordinate:", self.vertex_z_label)
        vertex_layout.addRow(self.vertex_frame_button)
        
        self.vertex_group.setLayout(vertex_layout)
        self.vertex_group.setVisible(False)  # Hidden by default
        layout.addWidget(self.vertex_group)

        # Create group box for frame properties
        self.frame_group = QGroupBox("Frame Information")
        frame_layout = QFormLayout()

        self.frame_name_label = QLabel("—")
        
        # Frame visibility checkbox
        self.frame_visible_checkbox = QCheckBox("Visible")
        self.frame_visible_checkbox.setChecked(True)
        self.frame_visible_checkbox.clicked.connect(self._on_frame_visibility_toggled)
        
        # Frame highlight button
        self.frame_highlight_button = QPushButton("Highlight Frame")
        self.frame_highlight_button.setCheckable(True)
        self.frame_highlight_button.clicked.connect(self._on_frame_highlight_toggled)
        
        # Frame Position Spinboxes
        self.frame_x_spin = QDoubleSpinBox()
        self.frame_x_spin.setRange(-1000.0, 1000.0)
        self.frame_x_spin.setSingleStep(0.01)
        self.frame_x_spin.setDecimals(4)
        self.frame_x_spin.valueChanged.connect(self._on_frame_position_changed)
        
        self.frame_y_spin = QDoubleSpinBox()
        self.frame_y_spin.setRange(-1000.0, 1000.0)
        self.frame_y_spin.setSingleStep(0.01)
        self.frame_y_spin.setDecimals(4)
        self.frame_y_spin.valueChanged.connect(self._on_frame_position_changed)
        
        self.frame_z_spin = QDoubleSpinBox()
        self.frame_z_spin.setRange(-1000.0, 1000.0)
        self.frame_z_spin.setSingleStep(0.01)
        self.frame_z_spin.setDecimals(4)
        self.frame_z_spin.valueChanged.connect(self._on_frame_position_changed)
        
        # Frame Rotation Spinboxes
        self.frame_rx_spin = QDoubleSpinBox()
        self.frame_rx_spin.setRange(-360.0, 360.0)
        self.frame_rx_spin.setSingleStep(1.0)
        self.frame_rx_spin.setDecimals(2)
        self.frame_rx_spin.setSuffix("°")
        self.frame_rx_spin.valueChanged.connect(self._on_frame_rotation_changed)

        self.frame_ry_spin = QDoubleSpinBox()
        self.frame_ry_spin.setRange(-360.0, 360.0)
        self.frame_ry_spin.setSingleStep(1.0)
        self.frame_ry_spin.setDecimals(2)
        self.frame_ry_spin.setSuffix("°")
        self.frame_ry_spin.valueChanged.connect(self._on_frame_rotation_changed)

        self.frame_rz_spin = QDoubleSpinBox()
        self.frame_rz_spin.setRange(-360.0, 360.0)
        self.frame_rz_spin.setSingleStep(1.0)
        self.frame_rz_spin.setDecimals(2)
        self.frame_rz_spin.setSuffix("°")
        self.frame_rz_spin.valueChanged.connect(self._on_frame_rotation_changed)

        self.frame_x_axis_label = QLabel("—")
        self.frame_y_axis_label = QLabel("—")
        self.frame_z_axis_label = QLabel("—")

        frame_layout.addRow("Frame Name:", self.frame_name_label)
        frame_layout.addRow("Visibility:", self.frame_visible_checkbox)
        frame_layout.addRow(self.frame_highlight_button)
        frame_layout.addRow("Position X (m):", self.frame_x_spin)
        frame_layout.addRow("Position Y (m):", self.frame_y_spin)
        frame_layout.addRow("Position Z (m):", self.frame_z_spin)
        frame_layout.addRow("Rotation X:", self.frame_rx_spin)
        frame_layout.addRow("Rotation Y:", self.frame_ry_spin)
        frame_layout.addRow("Rotation Z:", self.frame_rz_spin)
        frame_layout.addRow("X Axis:", self.frame_x_axis_label)
        frame_layout.addRow("Y Axis:", self.frame_y_axis_label)
        frame_layout.addRow("Z Axis:", self.frame_z_axis_label)

        self.frame_group.setLayout(frame_layout)
        self.frame_group.setVisible(False)
        layout.addWidget(self.frame_group)
        
        # Create group box for joint properties
        self.joint_group = QGroupBox("Joint Information")
        joint_layout = QFormLayout()
        
        self.joint_name_label = QLabel("—")
        self.joint_type_label = QLabel("—")
        self.joint_axis_label = QLabel("—")
        self.joint_body1_label = QLabel("—")
        self.joint_body2_label = QLabel("—")
        self.joint_frame_label = QLabel("—")
        
        # Motor properties
        self.joint_motorized_label = QLabel("—")
        self.joint_motor_type_label = QLabel("—")
        self.joint_motor_value_label = QLabel("—")
        
        joint_layout.addRow("Name:", self.joint_name_label)
        joint_layout.addRow("Type:", self.joint_type_label)
        joint_layout.addRow("Axis:", self.joint_axis_label)
        joint_layout.addRow("Body 1:", self.joint_body1_label)
        joint_layout.addRow("Body 2:", self.joint_body2_label)
        joint_layout.addRow("Joint Frame:", self.joint_frame_label)
        
        # Add separator for motor section
        separator_motor = QFrame()
        separator_motor.setFrameShape(QFrame.HLine)
        separator_motor.setFrameShadow(QFrame.Sunken)
        joint_layout.addRow(separator_motor)
        
        joint_layout.addRow("Motorized:", self.joint_motorized_label)
        joint_layout.addRow("Motor Type:", self.joint_motor_type_label)
        joint_layout.addRow("Motor Value:", self.joint_motor_value_label)
        
        self.joint_group.setLayout(joint_layout)
        self.joint_group.setVisible(False)
        layout.addWidget(self.joint_group)
        
        # Create group box for force properties
        self.force_group = QGroupBox("Force Information")
        force_layout = QFormLayout()
        
        self.force_name_label = QLabel("—")
        self.force_body_label = QLabel("—")
        self.force_frame_label = QLabel("—")
        self.force_magnitude_label = QLabel("—")
        self.force_direction_label = QLabel("—")
        
        force_layout.addRow("Name:", self.force_name_label)
        force_layout.addRow("Applied to Body:", self.force_body_label)
        force_layout.addRow("Application Frame:", self.force_frame_label)
        force_layout.addRow("Magnitude:", self.force_magnitude_label)
        force_layout.addRow("Direction:", self.force_direction_label)
        
        self.force_group.setLayout(force_layout)
        self.force_group.setVisible(False)
        layout.addWidget(self.force_group)
        
        # Create group box for torque properties
        self.torque_group = QGroupBox("Torque Information")
        torque_layout = QFormLayout()
        
        self.torque_name_label = QLabel("—")
        self.torque_body_label = QLabel("—")
        self.torque_frame_label = QLabel("—")
        self.torque_magnitude_label = QLabel("—")
        self.torque_axis_label = QLabel("—")
        
        torque_layout.addRow("Name:", self.torque_name_label)
        torque_layout.addRow("Applied to Body:", self.torque_body_label)
        torque_layout.addRow("Application Frame:", self.torque_frame_label)
        torque_layout.addRow("Magnitude:", self.torque_magnitude_label)
        torque_layout.addRow("Rotation Axis:", self.torque_axis_label)
        
        self.torque_group.setLayout(torque_layout)
        self.torque_group.setVisible(False)
        layout.addWidget(self.torque_group)
        
        # Create visualization options group
        self.viz_group = QGroupBox("Visualization Options")
        viz_layout = QVBoxLayout()
        
        # COM visibility checkbox
        self.com_checkbox = QCheckBox("Show Center of Mass")
        self.com_checkbox.setChecked(True)  # Enabled by default
        self.com_checkbox.stateChanged.connect(self._on_com_visibility_changed)
        viz_layout.addWidget(self.com_checkbox)
        
        # World frame visibility checkbox
        self.world_frame_checkbox = QCheckBox("Show World Frame")
        self.world_frame_checkbox.setChecked(False)  # Disabled by default
        self.world_frame_checkbox.stateChanged.connect(self._on_world_frame_visibility_changed)
        viz_layout.addWidget(self.world_frame_checkbox)
        
        # Local frame visibility checkbox
        self.local_frame_checkbox = QCheckBox("Show Local Frame")
        self.local_frame_checkbox.setChecked(True)  # Enabled by default
        self.local_frame_checkbox.setEnabled(False)  # Disabled until body selected
        self.local_frame_checkbox.stateChanged.connect(self._on_local_frame_visibility_changed)
        viz_layout.addWidget(self.local_frame_checkbox)
        
        self.viz_group.setLayout(viz_layout)
        layout.addWidget(self.viz_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
        
        # Initialize with no selection
        self.show_no_selection()
        
        print("Property panel initialized")
    
    def get_selection_mode(self) -> str:
        """Get the current selection mode"""
        return self.mode_dropdown.currentText()
    
    def set_selection_mode(self, mode: str):
        """
        Set the selection mode
        
        Args:
            mode: "Body", "Face", or "Edge"
        """
        self.mode_dropdown.blockSignals(True)
        self.mode_dropdown.setCurrentText(mode)
        self.mode_dropdown.blockSignals(False)
        self._update_group_visibility(mode)
    
    def _on_selection_mode_changed(self, mode: str):
        """Handle selection mode change"""
        self._update_group_visibility(mode)
        self.selection_mode_changed.emit(mode)
        print(f"Selection mode changed to: {mode}")
    
    def _update_group_visibility(self, mode: str):
        """Update visibility of property groups based on selection mode"""
        if mode == "Body":
            self.body_group.setVisible(True)
            self.face_group.setVisible(False)
            self.edge_group.setVisible(False)
            self.vertex_group.setVisible(False)
            # Keep frame group as-is (frame selection not tied to mode)
            self.face_frame_button.setEnabled(False)
        elif mode == "Face":
            self.body_group.setVisible(False)
            self.face_group.setVisible(True)
            self.edge_group.setVisible(False)
            self.vertex_group.setVisible(False)
            # Enabled once a face is selected
            self.face_frame_button.setEnabled(self.face_index_label.text() != "—")
            self.edge_frame_button.setEnabled(False)
        elif mode == "Edge":
            self.body_group.setVisible(False)
            self.face_group.setVisible(False)
            self.edge_group.setVisible(True)
            self.vertex_group.setVisible(False)
            self.face_frame_button.setEnabled(False)
            self.edge_frame_button.setEnabled(self.edge_index_label.text() != "—")
        elif mode == "Vertex":
            self.body_group.setVisible(False)
            self.face_group.setVisible(False)
            self.edge_group.setVisible(False)
            self.vertex_group.setVisible(True)
            self.face_frame_button.setEnabled(False)
            self.edge_frame_button.setEnabled(False)
            self.vertex_frame_button.setEnabled(self.vertex_index_label.text() != "—")
    
    def show_body_properties(self, body: RigidBody):
        """
        Display properties of the selected body
        
        Args:
            body: The RigidBody to display
        """
        # Force panel into body mode so fields are visible
        self.set_selection_mode("Body")
        self.body_group.setTitle("Body Information")
        self.name_label.setText(body.name)
        self.id_label.setText(str(body.id))
        
        # Set visibility checkbox state without triggering signal
        self.body_visible_checkbox.blockSignals(True)
        self.body_visible_checkbox.setChecked(body.visible)
        self.body_visible_checkbox.blockSignals(False)
        
        # Set contact detection checkbox state without triggering signal
        self.body_contact_checkbox.blockSignals(True)
        self.body_contact_checkbox.setChecked(body.contact_enabled)
        self.body_contact_checkbox.blockSignals(False)
        
        # Enable local frame checkbox when body is selected
        self.local_frame_checkbox.setEnabled(True)
        
        # Display volume with appropriate formatting
        if body.volume > 0:
            # Format volume with scientific notation for very small/large values
            if body.volume < 0.001 or body.volume > 1000:
                volume_text = f"{body.volume:.6e} m³"
            else:
                volume_text = f"{body.volume:.6f} m³"
            self.volume_label.setText(volume_text)
        else:
            self.volume_label.setText("Not calculated")
        
        # Display center of mass with appropriate formatting
        if body.center_of_mass is not None:
            com = body.center_of_mass
            com_text = f"[{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}] m"
            self.com_label.setText(com_text)
        else:
            self.com_label.setText("Not calculated")
        
        # Display inertia tensor with appropriate formatting
        if body.inertia_tensor is not None:
            I = body.inertia_tensor
            # Format as 3×3 matrix with scientific notation
            inertia_text = (
                f"[{I[0,0]:.4e}  {I[0,1]:.4e}  {I[0,2]:.4e}]\n"
                f"[{I[1,0]:.4e}  {I[1,1]:.4e}  {I[1,2]:.4e}]\n"
                f"[{I[2,0]:.4e}  {I[2,1]:.4e}  {I[2,2]:.4e}]\n"
                f"(kg·m²)"
            )
            self.inertia_text.setText(inertia_text)
        else:
            self.inertia_text.setText("Not calculated")
        
        print(f"Property panel updated for body: {body.name} (ID: {body.id})")
    
    def show_joint_properties(self, joint: Joint):
        """
        Display properties of the selected joint
        
        Args:
            joint: The Joint to display
        """
        self.joint_group.setTitle("Joint Information")
        
        self.joint_name_label.setText(joint.name)
        self.joint_type_label.setText(joint.joint_type.name)
        
        if joint.axis:
            self.joint_axis_label.setText(joint.axis)
        else:
            self.joint_axis_label.setText("—")
            
        self.joint_body1_label.setText(str(joint.body1_id))
        self.joint_body2_label.setText(str(joint.body2_id))
        self.joint_frame_label.setText(joint.frame.name)
        
        # Update motor properties
        if joint.is_motorized:
            self.joint_motorized_label.setText("Yes ⚡")
            self.joint_motorized_label.setStyleSheet("color: green; font-weight: bold;")
            self.joint_motor_type_label.setText(joint.motor_type.name)
            
            # Format motor value with appropriate units
            unit = ""
            if joint.motor_type.name == "VELOCITY":
                unit = "rad/s" if joint.joint_type == JointType.REVOLUTE else "m/s"
            elif joint.motor_type.name == "TORQUE":
                unit = "N·m" if joint.joint_type == JointType.REVOLUTE else "N"
            elif joint.motor_type.name == "POSITION":
                unit = "rad" if joint.joint_type == JointType.REVOLUTE else "m"
            
            self.joint_motor_value_label.setText(f"{joint.motor_value:.4f} {unit}")
        else:
            self.joint_motorized_label.setText("No")
            self.joint_motorized_label.setStyleSheet("")
            self.joint_motor_type_label.setText("—")
            self.joint_motor_value_label.setText("—")
        
        # Hide other groups
        self.body_group.setVisible(False)
        self.face_group.setVisible(False)
        self.edge_group.setVisible(False)
        self.frame_group.setVisible(False)
        self.joint_group.setVisible(True)
        
        print(f"Property panel updated for joint: {joint.name}")
    
    def show_force_properties(self, force: Force):
        """
        Display properties of the selected force
        
        Args:
            force: The Force to display
        """
        self.force_group.setTitle("Force Information")
        
        self.force_name_label.setText(force.name)
        self.force_body_label.setText(str(force.body_id))
        self.force_frame_label.setText(force.frame.name)
        self.force_magnitude_label.setText(f"{force.magnitude:.3f} N")
        
        # Format direction vector
        dir_str = f"[{force.direction[0]:.3f}, {force.direction[1]:.3f}, {force.direction[2]:.3f}]"
        self.force_direction_label.setText(dir_str)
        
        # Hide other groups
        self.body_group.setVisible(False)
        self.face_group.setVisible(False)
        self.edge_group.setVisible(False)
        self.frame_group.setVisible(False)
        self.joint_group.setVisible(False)
        self.force_group.setVisible(True)
        self.torque_group.setVisible(False)
        
        print(f"Property panel updated for force: {force.name}")
    
    def show_torque_properties(self, torque):
        """
        Display properties of the selected torque
        
        Args:
            torque: The Torque to display
        """
        self.torque_group.setTitle("Torque Information")
        
        self.torque_name_label.setText(torque.name)
        self.torque_body_label.setText(str(torque.body_id))
        self.torque_frame_label.setText(torque.frame.name)
        self.torque_magnitude_label.setText(f"{torque.magnitude:.3f} N·m")
        
        # Format axis vector
        axis_str = f"[{torque.axis[0]:.3f}, {torque.axis[1]:.3f}, {torque.axis[2]:.3f}]"
        self.torque_axis_label.setText(axis_str)
        
        # Hide other groups
        self.body_group.setVisible(False)
        self.face_group.setVisible(False)
        self.edge_group.setVisible(False)
        self.frame_group.setVisible(False)
        self.joint_group.setVisible(False)
        self.force_group.setVisible(False)
        self.torque_group.setVisible(True)
        
        print(f"Property panel updated for torque: {torque.name}")

    def show_no_selection(self):
        """Display message when nothing is selected"""
        self.body_group.setTitle("No Selection")
        self.name_label.setText("—")
        self.id_label.setText("—")
        self.volume_label.setText("—")
        self.com_label.setText("—")
        self.inertia_text.setText("—")
        
        # Disable local frame checkbox when no selection
        self.local_frame_checkbox.setEnabled(False)
        
        # Clear face properties
        self.face_index_label.setText("—")
        self.face_area_label.setText("—")
        self.face_center_label.setText("—")
        self.face_normal_label.setText("—")
        self.face_frame_button.setEnabled(False)
        
        # Clear edge properties
        self.edge_index_label.setText("—")
        self.edge_length_label.setText("—")
        self.edge_midpoint_label.setText("—")
        self.edge_direction_label.setText("—")
        self.edge_frame_button.setEnabled(False)
        
        # Clear vertex properties
        self.vertex_index_label.setText("—")
        self.vertex_x_label.setText("—")
        self.vertex_y_label.setText("—")
        self.vertex_z_label.setText("—")
        self.vertex_frame_button.setEnabled(False)

        # Clear frame properties
        self.frame_name_label.setText("—")
        self.frame_x_spin.blockSignals(True)
        self.frame_y_spin.blockSignals(True)
        self.frame_z_spin.blockSignals(True)
        self.frame_x_spin.setValue(0.0)
        self.frame_y_spin.setValue(0.0)
        self.frame_z_spin.setValue(0.0)
        self.frame_x_spin.blockSignals(False)
        self.frame_y_spin.blockSignals(False)
        self.frame_z_spin.blockSignals(False)
        
        self.frame_rx_spin.blockSignals(True)
        self.frame_ry_spin.blockSignals(True)
        self.frame_rz_spin.blockSignals(True)
        self.frame_rx_spin.setValue(0.0)
        self.frame_ry_spin.setValue(0.0)
        self.frame_rz_spin.setValue(0.0)
        self.frame_rx_spin.blockSignals(False)
        self.frame_ry_spin.blockSignals(False)
        self.frame_rz_spin.blockSignals(False)
        
        self.frame_x_axis_label.setText("—")
        self.frame_y_axis_label.setText("—")
        self.frame_z_axis_label.setText("—")
        self.frame_group.setVisible(False)
        self.force_group.setVisible(False)
        self.torque_group.setVisible(False)
        
    def clear(self):
        """Clear all displayed properties"""
        self.show_no_selection()
    
    def _on_com_visibility_changed(self, state):
        """Handle COM visibility checkbox state change"""
        is_visible = (state == Qt.CheckState.Checked.value)
        self.com_visibility_changed.emit(is_visible)
        print(f"COM visibility changed: {'shown' if is_visible else 'hidden'}")
    
    def _on_world_frame_visibility_changed(self, state):
        """Handle world frame visibility checkbox state change"""
        is_visible = (state == Qt.CheckState.Checked.value)
        self.world_frame_visibility_changed.emit(is_visible)
        print(f"World frame visibility changed: {'shown' if is_visible else 'hidden'}")
    
    def _on_local_frame_visibility_changed(self, state):
        """Handle local frame visibility checkbox state change"""
        is_visible = (state == Qt.CheckState.Checked.value)
        self.local_frame_visibility_changed.emit(is_visible)
        print(f"Local frame visibility changed: {'shown' if is_visible else 'hidden'}")
    
    def show_face_properties(self, face_properties):
        """
        Display properties of the selected face
        
        Args:
            face_properties: FaceProperties object
        """
        # Force panel into face mode so fields are visible
        self.set_selection_mode("Face")
        self.face_group.setTitle("Face Information")
        self.face_index_label.setText(str(face_properties.face_index))
        
        # Display area
        if face_properties.area > 0:
            if face_properties.area < 0.001 or face_properties.area > 1000:
                area_text = f"{face_properties.area:.6e} m²"
            else:
                area_text = f"{face_properties.area:.6f} m²"
            self.face_area_label.setText(area_text)
        else:
            self.face_area_label.setText("Not calculated")
        
        # Display center
        center = face_properties.center
        center_text = f"[{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}] m"
        self.face_center_label.setText(center_text)
        
        # Display normal
        normal = face_properties.normal
        normal_text = f"[{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]"
        self.face_normal_label.setText(normal_text)
        self.face_frame_button.setEnabled(True)
        
        print(f"Property panel updated for face: {face_properties}")
    
    def show_edge_properties(self, edge_properties):
        """
        Display properties of the selected edge
        
        Args:
            edge_properties: EdgeProperties object
        """
        # Force panel into edge mode so fields are visible
        self.set_selection_mode("Edge")
        self.edge_group.setTitle("Edge Information")
        self.edge_index_label.setText(str(edge_properties.edge_index))
        
        # Display length
        if edge_properties.length > 0:
            if edge_properties.length < 0.001 or edge_properties.length > 1000:
                length_text = f"{edge_properties.length:.6e} m"
            else:
                length_text = f"{edge_properties.length:.6f} m"
            self.edge_length_label.setText(length_text)
        else:
            self.edge_length_label.setText("Not calculated")
        
        # Display midpoint
        midpoint = edge_properties.midpoint
        midpoint_text = f"[{midpoint[0]:.6f}, {midpoint[1]:.6f}, {midpoint[2]:.6f}] m"
        self.edge_midpoint_label.setText(midpoint_text)
        
        # Display direction
        direction = edge_properties.direction
        direction_text = f"[{direction[0]:.6f}, {direction[1]:.6f}, {direction[2]:.6f}]"
        self.edge_direction_label.setText(direction_text)
        self.edge_frame_button.setEnabled(True)
        
        print(f"Property panel updated for edge: {edge_properties}")
    
    def show_vertex_properties(self, vertex_properties: VertexProperties):
        """
        Display properties of the selected vertex
        
        Args:
            vertex_properties: VertexProperties object
        """
        # Force panel into vertex mode so fields are visible
        self.set_selection_mode("Vertex")
        self.vertex_group.setTitle("Vertex Information")
        self.vertex_index_label.setText(str(vertex_properties.vertex_index))
        
        # Display coordinates
        coords = vertex_properties.coordinates
        self.vertex_x_label.setText(f"{coords[0]:.6f} m")
        self.vertex_y_label.setText(f"{coords[1]:.6f} m")
        self.vertex_z_label.setText(f"{coords[2]:.6f} m")
        self.vertex_frame_button.setEnabled(True)
        
        print(f"Property panel updated for vertex: {vertex_properties}")

    def show_frame_properties(self, frame):
        """Display properties of a frame"""
        self.frame_group.setTitle("Frame Information")
        
        # Block signals to prevent feedback loop
        self.frame_x_spin.blockSignals(True)
        self.frame_y_spin.blockSignals(True)
        self.frame_z_spin.blockSignals(True)
        self.frame_rx_spin.blockSignals(True)
        self.frame_ry_spin.blockSignals(True)
        self.frame_rz_spin.blockSignals(True)
        
        self.frame_name_label.setText(frame.name)
        origin = frame.origin
        self.frame_x_spin.setValue(origin[0])
        self.frame_y_spin.setValue(origin[1])
        self.frame_z_spin.setValue(origin[2])
        
        # rotation
        euler = frame.get_euler_angles()
        self.frame_rx_spin.setValue(euler[0])
        self.frame_ry_spin.setValue(euler[1])
        self.frame_rz_spin.setValue(euler[2])
        
        rot = frame.rotation_matrix
        self.frame_x_axis_label.setText(f"[{rot[0,0]:.6f}, {rot[1,0]:.6f}, {rot[2,0]:.6f}]")
        self.frame_y_axis_label.setText(f"[{rot[0,1]:.6f}, {rot[1,1]:.6f}, {rot[2,1]:.6f}]")
        self.frame_z_axis_label.setText(f"[{rot[0,2]:.6f}, {rot[1,2]:.6f}, {rot[2,2]:.6f}]")
        
        # Unblock signals
        self.frame_x_spin.blockSignals(False)
        self.frame_y_spin.blockSignals(False)
        self.frame_z_spin.blockSignals(False)
        self.frame_rx_spin.blockSignals(False)
        self.frame_ry_spin.blockSignals(False)
        self.frame_rz_spin.blockSignals(False)
        
        self.frame_group.setVisible(True)

    def _on_frame_position_changed(self):
        """Handle numeric input changes for frame position"""
        frame_name = self.frame_name_label.text()
        if frame_name != "—":
            x = self.frame_x_spin.value()
            y = self.frame_y_spin.value()
            z = self.frame_z_spin.value()
            self.frame_position_changed.emit(frame_name, (x, y, z))

    def _on_frame_rotation_changed(self):
        """Handle numeric input changes for frame rotation"""
        frame_name = self.frame_name_label.text()
        if frame_name != "—":
            rx = self.frame_rx_spin.value()
            ry = self.frame_ry_spin.value()
            rz = self.frame_rz_spin.value()
            self.frame_rotation_changed.emit(frame_name, (rx, ry, rz))

    def _on_body_visibility_toggled(self, checked):
        """Handle body visibility toggle"""
        try:
            body_id = int(self.id_label.text())
            self.body_visibility_changed.emit(body_id, checked)
        except ValueError:
            pass  # ID not an integer (e.g. "—")
    
    def _on_contact_detection_toggled(self, checked):
        """Handle contact detection toggle"""
        try:
            body_id = int(self.id_label.text())
            self.contact_detection_changed.emit(body_id, checked)
        except ValueError:
            pass  # ID not an integer (e.g. "—")

    def _on_create_frame_from_face(self):
        """Emit request to create a frame from the currently selected face"""
        self.create_frame_from_face.emit()

    def _on_create_frame_from_edge(self):
        """Emit request to create a frame from the currently selected edge"""
        self.create_frame_from_edge.emit()
    
    def _on_create_frame_from_vertex(self):
        """Emit request to create a frame from the currently selected vertex"""
        self.create_frame_from_vertex.emit()

    def _on_frame_visibility_toggled(self, checked):
        """Handle frame visibility toggle"""
        frame_name = self.frame_name_label.text()
        if frame_name != "—":
            self.frame_visibility_changed.emit(frame_name, checked)
    
    def _on_frame_highlight_toggled(self, checked):
        """Handle frame highlight toggle"""
        frame_name = self.frame_name_label.text()
        if frame_name != "—":
            self.frame_highlight_changed.emit(frame_name, checked)
            # Update button text
            if checked:
                self.frame_highlight_button.setText("Unhighlight Frame")
            else:
                self.frame_highlight_button.setText("Highlight Frame")
    
    def _on_all_frames_visibility_toggled(self, checked):
        """Handle all frames visibility toggle"""
        self.all_frames_visibility_changed.emit(checked)
