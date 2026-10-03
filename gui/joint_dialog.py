"""Joint editor for choosing one attachment frame on each connected body."""

from typing import Dict, List, Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFormLayout, QLabel, QLineEdit,
    QPushButton, QVBoxLayout,
)

from core.data_structures import JointType, RigidBody
from core.joint_factory import AXES, AXIAL_JOINTS, JointFrameOption


class JointCreationDialog(QDialog):
    preview_changed = Signal(object, object)

    def __init__(self, bodies: List[RigidBody], frames_by_body: Dict[int, List[JointFrameOption]], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Joint")
        self.resize(520, 590)
        self.bodies = list(bodies)
        self.frames_by_body = frames_by_body
        self.assemble_after_create = False

        layout = QVBoxLayout(self)
        form = QFormLayout()
        layout.addLayout(form)

        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter joint name")
        form.addRow("Joint Name:", self.name_input)

        self.type_combo = QComboBox()
        for joint_type in JointType:
            self.type_combo.addItem(joint_type.name, joint_type)
        form.addRow("Joint Type:", self.type_combo)

        self.body1_combo = self._body_combo(0)
        self.body2_combo = self._body_combo(1 if len(self.bodies) > 1 else 0)
        form.addRow("Body 1:", self.body1_combo)
        self.frame1_combo = QComboBox()
        form.addRow("Frame on Body 1:", self.frame1_combo)
        self.axis1_combo = self._axis_combo()
        form.addRow("Axis on Body 1:", self.axis1_combo)
        self.flip1_checkbox = QCheckBox("Flip frame 180°")
        form.addRow("Body 1 Orientation:", self.flip1_checkbox)

        form.addRow(QLabel("Connect to"))
        form.addRow("Body 2:", self.body2_combo)
        self.frame2_combo = QComboBox()
        form.addRow("Frame on Body 2:", self.frame2_combo)
        self.axis2_combo = self._axis_combo()
        form.addRow("Axis on Body 2:", self.axis2_combo)
        self.flip2_checkbox = QCheckBox("Flip frame 180°")
        form.addRow("Body 2 Orientation:", self.flip2_checkbox)

        self.help_label = QLabel()
        self.help_label.setWordWrap(True)
        form.addRow(self.help_label)
        self.reference_body_checkbox = QCheckBox("Keep Body 1 fixed during assembly")
        form.addRow(self.reference_body_checkbox)

        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.create_button = QPushButton("Create")
        self.assemble_button = QPushButton("Create & Assemble")
        buttons.addButton(self.create_button, QDialogButtonBox.AcceptRole)
        buttons.addButton(self.assemble_button, QDialogButtonBox.AcceptRole)
        buttons.rejected.connect(self.reject)
        self.create_button.clicked.connect(self._accept_without_assembly)
        self.assemble_button.clicked.connect(self._accept_with_assembly)
        layout.addWidget(buttons)

        self.type_combo.currentIndexChanged.connect(self._update_type_controls)
        self.type_combo.currentIndexChanged.connect(self._preview)
        self.body1_combo.currentIndexChanged.connect(lambda _index: self._refresh_frames(1))
        self.body2_combo.currentIndexChanged.connect(lambda _index: self._refresh_frames(2))
        self.body1_combo.currentIndexChanged.connect(self._update_reference_control)
        for combo in (self.frame1_combo, self.frame2_combo, self.axis1_combo, self.axis2_combo):
            combo.currentIndexChanged.connect(self._preview)
        self.flip1_checkbox.toggled.connect(self._preview)
        self.flip2_checkbox.toggled.connect(self._preview)
        self.name_input.textChanged.connect(self._update_buttons)
        self._refresh_frames(1)
        self._refresh_frames(2)
        self._update_type_controls()
        self._update_reference_control()
        self._update_buttons()

    def _body_combo(self, initial_index: int) -> QComboBox:
        combo = QComboBox()
        for body in self.bodies:
            combo.addItem(f"{body.name} (ID: {body.id})", int(body.id))
        if combo.count():
            combo.setCurrentIndex(min(initial_index, combo.count() - 1))
        return combo

    @staticmethod
    def _axis_combo() -> QComboBox:
        combo = QComboBox()
        combo.addItems(AXES)
        combo.setCurrentText("+Z")
        return combo

    def _refresh_frames(self, side: int) -> None:
        body_combo = self.body1_combo if side == 1 else self.body2_combo
        frame_combo = self.frame1_combo if side == 1 else self.frame2_combo
        selected_body = body_combo.currentData()
        previous_key = frame_combo.currentData()
        frame_combo.blockSignals(True)
        frame_combo.clear()
        for option in self.frames_by_body.get(int(selected_body), []):
            frame_combo.addItem(option.label, option)
            if option.key == getattr(previous_key, "key", None):
                frame_combo.setCurrentIndex(frame_combo.count() - 1)
        frame_combo.blockSignals(False)
        self._update_buttons()
        self._preview()

    def _update_type_controls(self) -> None:
        joint_type = self.type_combo.currentData()
        enabled = joint_type in AXIAL_JOINTS
        self.axis1_combo.setEnabled(enabled)
        self.axis2_combo.setEnabled(enabled)
        self.help_label.setText(
            "Each selected frame stays attached to its body. Create stores the joint; "
            "Create & Assemble also asks the solver to bring the assembly into position."
        )
        self._update_default_name()

    def _update_default_name(self) -> None:
        if not self.name_input.text() or self.name_input.text().startswith("Joint_"):
            self.name_input.setText(f"Joint_{self.type_combo.currentText()}")

    def _update_buttons(self) -> None:
        complete = (
            bool(self.name_input.text().strip())
            and self.body1_combo.currentData() is not None
            and self.body2_combo.currentData() is not None
            and self.body1_combo.currentData() != self.body2_combo.currentData()
            and self.frame1_combo.currentData() is not None
            and self.frame2_combo.currentData() is not None
        )
        self.create_button.setEnabled(complete)
        self.assemble_button.setEnabled(complete)

    def _update_reference_control(self, *_args) -> None:
        self.reference_body_checkbox.setVisible(self.body1_combo.currentData() != -1)

    def _preview(self, *_args) -> None:
        self.preview_changed.emit(self.frame1_combo.currentData(), self.frame2_combo.currentData())

    def _accept_without_assembly(self) -> None:
        self.assemble_after_create = False
        self.accept()

    def _accept_with_assembly(self) -> None:
        self.assemble_after_create = True
        self.accept()

    def get_data(self):
        """Return both independently selected body attachments and their axes."""
        return {
            "name": self.name_input.text().strip(),
            "joint_type": self.type_combo.currentData(),
            "body1_id": self.body1_combo.currentData(),
            "body2_id": self.body2_combo.currentData(),
            "frame1": self.frame1_combo.currentData(),
            "frame2": self.frame2_combo.currentData(),
            "axis1": self.axis1_combo.currentText(),
            "axis2": self.axis2_combo.currentText(),
            "flip1": self.flip1_checkbox.isChecked(),
            "flip2": self.flip2_checkbox.isChecked(),
            "assemble": self.assemble_after_create,
            "reference_body_id": (
                self.body1_combo.currentData()
                if self.reference_body_checkbox.isChecked()
                and self.body1_combo.currentData() != -1
                else None
            ),
        }
