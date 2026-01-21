"""
Dialog for adding motor actuation to joints
"""
from PySide6.QtWidgets import (QDialog, QFormLayout, QComboBox, 
                               QDialogButtonBox, QVBoxLayout, QLabel, QDoubleSpinBox,
                               QGroupBox)
from PySide6.QtCore import Qt
from core.data_structures import Joint, JointType, MotorType
from typing import List


class MotorDialog(QDialog):
    """Dialog for adding a motor to a joint"""
    
    def __init__(self, joints: List[Joint], selected_joint_name: str = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Motor to Joint")
        self.resize(450, 300)
        
        self.joints = joints
        self.selected_joint_name = selected_joint_name
        
        # Filter to only motorizable joints (revolute and prismatic that aren't already motorized)
        self.motorizable_joints = [j for j in joints 
                                   if j.joint_type in [JointType.REVOLUTE, JointType.PRISMATIC]
                                   and not j.is_motorized]
        
        # Main layout
        self.layout = QVBoxLayout(self)
        self.form_layout = QFormLayout()
        
        # Joint Selection Dropdown
        self.joint_combo = QComboBox()
        if not self.motorizable_joints:
            self.joint_combo.addItem("No available joints", None)
        else:
            for joint in self.motorizable_joints:
                display_text = f"{joint.name} ({joint.joint_type.name})"
                self.joint_combo.addItem(display_text, joint)
            
            # Pre-select joint if provided
            if selected_joint_name:
                for i in range(self.joint_combo.count()):
                    joint = self.joint_combo.itemData(i)
                    if joint and joint.name == selected_joint_name:
                        self.joint_combo.setCurrentIndex(i)
                        break
        
        self.form_layout.addRow("Joint:", self.joint_combo)
        
        # Motor Type Dropdown
        self.motor_type_combo = QComboBox()
        for motor_type in MotorType:
            self.motor_type_combo.addItem(motor_type.name, motor_type)
        
        self.form_layout.addRow("Motor Type:", self.motor_type_combo)
        
        # Connect signal to update units when motor type or joint changes
        self.motor_type_combo.currentIndexChanged.connect(self.update_value_units)
        self.joint_combo.currentIndexChanged.connect(self.update_value_units)
        
        self.layout.addLayout(self.form_layout)
        
        # Motor Value Group
        value_group = QGroupBox("Motor Value")
        value_layout = QFormLayout()
        
        self.value_spin = QDoubleSpinBox()
        self.value_spin.setRange(-1e6, 1e6)
        self.value_spin.setDecimals(4)
        self.value_spin.setValue(10.0)
        self.value_spin.setSingleStep(1.0)
        
        value_layout.addRow("Value:", self.value_spin)
        
        # Units label
        self.units_label = QLabel()
        value_layout.addRow("Units:", self.units_label)
        
        # Description label
        self.description_label = QLabel()
        self.description_label.setWordWrap(True)
        self.description_label.setStyleSheet("color: gray; font-style: italic;")
        value_layout.addRow("", self.description_label)
        
        value_group.setLayout(value_layout)
        self.layout.addWidget(value_group)
        
        # Update units initially
        self.update_value_units()
        
        # Dialog buttons
        self.button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        
        # Disable OK if no joints available
        if not self.motorizable_joints:
            self.button_box.button(QDialogButtonBox.Ok).setEnabled(False)
        
        self.layout.addWidget(self.button_box)
    
    def update_value_units(self):
        """Update the units label and description based on selected motor type and joint type"""
        motor_type = self.motor_type_combo.currentData()
        joint = self.joint_combo.currentData()
        
        if not joint or not motor_type:
            self.units_label.setText("N/A")
            self.description_label.setText("")
            return
        
        is_revolute = joint.joint_type == JointType.REVOLUTE
        
        if motor_type == MotorType.VELOCITY:
            if is_revolute:
                self.units_label.setText("rad/s")
                self.description_label.setText("Angular velocity (positive = counterclockwise)")
            else:
                self.units_label.setText("m/s")
                self.description_label.setText("Linear velocity along joint axis")
        
        elif motor_type == MotorType.TORQUE:
            if is_revolute:
                self.units_label.setText("N·m")
                self.description_label.setText("Applied torque (positive = counterclockwise)")
            else:
                self.units_label.setText("N")
                self.description_label.setText("Applied force along joint axis")
        
        elif motor_type == MotorType.POSITION:
            if is_revolute:
                self.units_label.setText("rad")
                self.description_label.setText("Target angular position")
            else:
                self.units_label.setText("m")
                self.description_label.setText("Target linear position")
    
    def get_motor_data(self):
        """
        Get the selected motor configuration
        
        Returns:
            Tuple of (joint, motor_type, value) or (None, None, None) if invalid
        """
        joint = self.joint_combo.currentData()
        motor_type = self.motor_type_combo.currentData()
        value = self.value_spin.value()
        
        if joint and motor_type:
            return joint, motor_type, value
        
        return None, None, None
