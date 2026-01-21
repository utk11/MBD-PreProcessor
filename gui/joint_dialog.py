from PySide6.QtWidgets import (QDialog, QFormLayout, QLineEdit, QComboBox, 
                               QDialogButtonBox, QVBoxLayout, QLabel)
from core.data_structures import JointType, RigidBody, Frame
from typing import List, Dict

class JointCreationDialog(QDialog):
    def __init__(self, bodies: List[RigidBody], frames: List[Frame], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Joint")
        self.resize(400, 300)
        
        self.bodies = bodies
        self.frames = frames
        
        # Layout
        self.layout = QVBoxLayout(self)
        self.form_layout = QFormLayout()
        
        # Name Input
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter joint name")
        self.form_layout.addRow("Joint Name:", self.name_input)
        
        # Joint Type Dropdown
        self.type_combo = QComboBox()
        for jt in JointType:
            self.type_combo.addItem(jt.name, jt)
        self.form_layout.addRow("Joint Type:", self.type_combo)

        # Axis Dropdown
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["+Z", "-Z", "+X", "-X", "+Y", "-Y"])
        self.form_layout.addRow("Axis:", self.axis_combo)
        
        # Body 1 Dropdown
        self.body1_combo = QComboBox()
        for body in self.bodies:
            self.body1_combo.addItem(f"{body.name} (ID: {body.id})", body.id)
        self.form_layout.addRow("Body 1:", self.body1_combo)
        
        # Body 2 Dropdown
        self.body2_combo = QComboBox()
        for body in self.bodies:
            self.body2_combo.addItem(f"{body.name} (ID: {body.id})", body.id)
        # Select second body by default if possible
        if len(self.bodies) > 1:
            self.body2_combo.setCurrentIndex(1)
        self.form_layout.addRow("Body 2:", self.body2_combo)
        
        # Joint Frame Dropdown (single frame in global coordinates)
        self.frame_combo = QComboBox()
        for frame in self.frames:
            self.frame_combo.addItem(frame.name, frame)
        self.form_layout.addRow("Joint Frame (Global):", self.frame_combo)
        
        self.layout.addLayout(self.form_layout)
        
        # Buttons
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addWidget(self.button_box)
        
        # Pre-generate name based on type
        self.type_combo.currentIndexChanged.connect(self._update_default_name)
        self.type_combo.currentIndexChanged.connect(self._update_axis_visibility)
        self._update_default_name()
        self._update_axis_visibility()

    def _update_axis_visibility(self):
        jt = self.type_combo.currentData()
        # Fixed and Spherical usually don't have a single axis
        # Revolute, Prismatic, Cylindrical DO have an axis
        has_axis = jt in [JointType.REVOLUTE, JointType.PRISMATIC, JointType.CYLINDRICAL]
        self.axis_combo.setEnabled(has_axis)

    def _update_default_name(self):
        jt_name = self.type_combo.currentText()
        if not self.name_input.text() or self.name_input.text().startswith("Joint_"):
             self.name_input.setText(f"Joint_{jt_name}")

    def get_data(self):
        """Return the collected data"""
        name = self.name_input.text()
        joint_type = self.type_combo.currentData()
        body1_id = self.body1_combo.currentData()
        body2_id = self.body2_combo.currentData()
        frame = self.frame_combo.currentData()
        axis = self.axis_combo.currentText()
        
        return name, joint_type, body1_id, body2_id, frame, axis
