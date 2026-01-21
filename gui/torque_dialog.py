"""
Dialog for creating external torques (moments) applied to bodies
"""
from PySide6.QtWidgets import (QDialog, QFormLayout, QLineEdit, QComboBox, 
                               QDialogButtonBox, QVBoxLayout, QLabel, QDoubleSpinBox,
                               QGroupBox, QRadioButton, QHBoxLayout)
from PySide6.QtCore import Qt
from core.data_structures import RigidBody, Frame
from typing import List
import numpy as np


class TorqueDialog(QDialog):
    """Dialog for creating a torque on a body"""
    
    def __init__(self, bodies: List[RigidBody], frames: List[Frame], 
                 selected_body_id: int = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Torque")
        self.resize(450, 400)
        
        self.bodies = bodies
        self.frames = frames
        self.selected_body_id = selected_body_id
        
        # Main layout
        self.layout = QVBoxLayout(self)
        self.form_layout = QFormLayout()
        
        # Name Input
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter torque name")
        self.name_input.setText("Torque_1")
        self.form_layout.addRow("Torque Name:", self.name_input)
        
        # Body Dropdown
        self.body_combo = QComboBox()
        for body in self.bodies:
            self.body_combo.addItem(f"{body.name} (ID: {body.id})", body.id)
        
        # Pre-select body if provided
        if selected_body_id is not None:
            for i in range(self.body_combo.count()):
                if self.body_combo.itemData(i) == selected_body_id:
                    self.body_combo.setCurrentIndex(i)
                    break
        
        self.form_layout.addRow("Applied to Body:", self.body_combo)
        
        # Frame Dropdown
        self.frame_combo = QComboBox()
        for frame in self.frames:
            self.frame_combo.addItem(frame.name, frame)
        self.form_layout.addRow("Application Frame:", self.frame_combo)
        
        # Magnitude Input
        self.magnitude_spin = QDoubleSpinBox()
        self.magnitude_spin.setRange(0.0, 1e9)
        self.magnitude_spin.setDecimals(3)
        self.magnitude_spin.setValue(10.0)
        self.magnitude_spin.setSuffix(" N·m")
        self.form_layout.addRow("Magnitude:", self.magnitude_spin)
        
        self.layout.addLayout(self.form_layout)
        
        # Axis Direction Group
        axis_group = QGroupBox("Rotation Axis")
        axis_layout = QVBoxLayout()
        
        # Radio buttons for axis mode
        self.frame_axis_radio = QRadioButton("Use Frame Axis")
        self.custom_vector_radio = QRadioButton("Custom Vector")
        self.frame_axis_radio.setChecked(True)
        
        axis_layout.addWidget(self.frame_axis_radio)
        axis_layout.addWidget(self.custom_vector_radio)
        
        # Frame axis dropdown
        frame_axis_layout = QHBoxLayout()
        frame_axis_layout.addWidget(QLabel("  Axis:"))
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["+X", "-X", "+Y", "-Y", "+Z", "-Z"])
        self.axis_combo.setCurrentText("+Z")
        frame_axis_layout.addWidget(self.axis_combo)
        frame_axis_layout.addStretch()
        axis_layout.addLayout(frame_axis_layout)
        
        # Custom vector input
        custom_vector_layout = QHBoxLayout()
        custom_vector_layout.addWidget(QLabel("  Vector:"))
        
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setDecimals(3)
        self.x_spin.setValue(0.0)
        self.x_spin.setPrefix("x: ")
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setDecimals(3)
        self.y_spin.setValue(0.0)
        self.y_spin.setPrefix("y: ")
        
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-1000, 1000)
        self.z_spin.setDecimals(3)
        self.z_spin.setValue(1.0)
        self.z_spin.setPrefix("z: ")
        
        custom_vector_layout.addWidget(self.x_spin)
        custom_vector_layout.addWidget(self.y_spin)
        custom_vector_layout.addWidget(self.z_spin)
        custom_vector_layout.addStretch()
        axis_layout.addLayout(custom_vector_layout)
        
        axis_group.setLayout(axis_layout)
        self.layout.addWidget(axis_group)
        
        # Connect radio buttons to enable/disable inputs
        self.frame_axis_radio.toggled.connect(self._on_mode_changed)
        self._on_mode_changed()
        
        # Dialog Buttons
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addWidget(self.button_box)
    
    def _on_mode_changed(self):
        """Enable/disable inputs based on selected mode"""
        frame_mode = self.frame_axis_radio.isChecked()
        self.axis_combo.setEnabled(frame_mode)
        self.x_spin.setEnabled(not frame_mode)
        self.y_spin.setEnabled(not frame_mode)
        self.z_spin.setEnabled(not frame_mode)
    
    def get_axis_vector(self) -> np.ndarray:
        """Get the rotation axis vector based on selected mode"""
        if self.frame_axis_radio.isChecked():
            # Use frame axis - transform by the selected frame's rotation
            axis_str = self.axis_combo.currentText()
            axis_map = {
                "+X": np.array([1.0, 0.0, 0.0]),
                "-X": np.array([-1.0, 0.0, 0.0]),
                "+Y": np.array([0.0, 1.0, 0.0]),
                "-Y": np.array([0.0, -1.0, 0.0]),
                "+Z": np.array([0.0, 0.0, 1.0]),
                "-Z": np.array([0.0, 0.0, -1.0])
            }
            local_axis = axis_map[axis_str]
            
            # Transform by the frame's rotation matrix to get world-space axis
            frame = self.frame_combo.currentData()
            if frame and hasattr(frame, 'rotation_matrix'):
                world_axis = frame.rotation_matrix @ local_axis
                return world_axis
            else:
                return local_axis
        else:
            # Use custom vector (assumed to be in world coordinates)
            return np.array([
                self.x_spin.value(),
                self.y_spin.value(),
                self.z_spin.value()
            ])
    
    def get_data(self):
        """Return the torque data as tuple"""
        name = self.name_input.text().strip()
        body_id = self.body_combo.currentData()
        frame = self.frame_combo.currentData()
        magnitude = self.magnitude_spin.value()
        axis = self.get_axis_vector()
        
        return name, body_id, frame, magnitude, axis
