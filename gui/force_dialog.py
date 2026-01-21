"""
Dialog for creating external forces applied to bodies
"""
from PySide6.QtWidgets import (QDialog, QFormLayout, QLineEdit, QComboBox, 
                               QDialogButtonBox, QVBoxLayout, QLabel, QDoubleSpinBox,
                               QGroupBox, QRadioButton, QHBoxLayout)
from PySide6.QtCore import Qt
from core.data_structures import RigidBody, Frame
from typing import List
import numpy as np


class ForceDialog(QDialog):
    """Dialog for creating a force on a body"""
    
    def __init__(self, bodies: List[RigidBody], frames: List[Frame], 
                 selected_body_id: int = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Force")
        self.resize(450, 400)
        
        self.bodies = bodies
        self.frames = frames
        self.selected_body_id = selected_body_id
        
        # Main layout
        self.layout = QVBoxLayout(self)
        self.form_layout = QFormLayout()
        
        # Name Input
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter force name")
        self.name_input.setText("Force_1")
        self.form_layout.addRow("Force Name:", self.name_input)
        
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
        self.magnitude_spin.setValue(100.0)
        self.magnitude_spin.setSuffix(" N")
        self.form_layout.addRow("Magnitude:", self.magnitude_spin)
        
        self.layout.addLayout(self.form_layout)
        
        # Direction Group
        direction_group = QGroupBox("Direction")
        direction_layout = QVBoxLayout()
        
        # Radio buttons for direction mode
        self.frame_axis_radio = QRadioButton("Use Frame Axis")
        self.custom_vector_radio = QRadioButton("Custom Vector")
        self.frame_axis_radio.setChecked(True)
        
        direction_layout.addWidget(self.frame_axis_radio)
        direction_layout.addWidget(self.custom_vector_radio)
        
        # Frame axis dropdown
        axis_layout = QHBoxLayout()
        axis_layout.addWidget(QLabel("  Axis:"))
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["+X", "-X", "+Y", "-Y", "+Z", "-Z"])
        self.axis_combo.setCurrentText("+Z")
        axis_layout.addWidget(self.axis_combo)
        axis_layout.addStretch()
        direction_layout.addLayout(axis_layout)
        
        # Custom vector inputs
        vector_layout = QHBoxLayout()
        vector_layout.addWidget(QLabel("  Vector:"))
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1e6, 1e6)
        self.x_spin.setDecimals(3)
        self.x_spin.setValue(0.0)
        self.x_spin.setPrefix("X: ")
        self.x_spin.setEnabled(False)
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1e6, 1e6)
        self.y_spin.setDecimals(3)
        self.y_spin.setValue(0.0)
        self.y_spin.setPrefix("Y: ")
        self.y_spin.setEnabled(False)
        
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-1e6, 1e6)
        self.z_spin.setDecimals(3)
        self.z_spin.setValue(1.0)
        self.z_spin.setPrefix("Z: ")
        self.z_spin.setEnabled(False)
        
        vector_layout.addWidget(self.x_spin)
        vector_layout.addWidget(self.y_spin)
        vector_layout.addWidget(self.z_spin)
        direction_layout.addLayout(vector_layout)
        
        direction_group.setLayout(direction_layout)
        self.layout.addWidget(direction_group)
        
        # Connect radio buttons
        self.frame_axis_radio.toggled.connect(self._on_direction_mode_changed)
        
        # Buttons
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addWidget(self.button_box)
    
    def _on_direction_mode_changed(self, checked):
        """Toggle between frame axis and custom vector mode"""
        if checked:  # Frame axis mode
            self.axis_combo.setEnabled(True)
            self.x_spin.setEnabled(False)
            self.y_spin.setEnabled(False)
            self.z_spin.setEnabled(False)
        else:  # Custom vector mode
            self.axis_combo.setEnabled(False)
            self.x_spin.setEnabled(True)
            self.y_spin.setEnabled(True)
            self.z_spin.setEnabled(True)
    
    def get_direction_vector(self) -> np.ndarray:
        """Get the direction vector based on selected mode"""
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
            local_direction = axis_map[axis_str]
            
            # Transform by the frame's rotation matrix to get world-space direction
            frame = self.frame_combo.currentData()
            if frame and hasattr(frame, 'rotation_matrix'):
                world_direction = frame.rotation_matrix @ local_direction
                return world_direction
            else:
                return local_direction
        else:
            # Use custom vector (assumed to be in world coordinates)
            return np.array([
                self.x_spin.value(),
                self.y_spin.value(),
                self.z_spin.value()
            ])
    
    def get_data(self):
        """Return the collected data"""
        name = self.name_input.text()
        body_id = self.body_combo.currentData()
        frame = self.frame_combo.currentData()
        magnitude = self.magnitude_spin.value()
        direction = self.get_direction_vector()
        
        return name, body_id, frame, magnitude, direction
