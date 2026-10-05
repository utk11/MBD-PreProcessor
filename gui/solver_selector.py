"""Visible linear-method selection for assembly solving and dragging."""

from PySide6.QtWidgets import QComboBox


SOLVER_CHOICES = (
    ("dense", "Dense (default)"),
    ("superlu", "SuperLU"),
    ("cg", "Conjugate Gradient (CG)"),
    ("lsmr", "LSMR"),
)


class SolverSelector(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("linearSolverSelector")
        self.setAccessibleName("Linear solver")
        self.setToolTip("Choose the linear method used for dragging and Solve Assembly.")
        for key, label in SOLVER_CHOICES:
            self.addItem(label, key)
