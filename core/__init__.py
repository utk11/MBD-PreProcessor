"""
Core functionality for Multi-Body Dynamics Preprocessor
"""

from .data_structures import RigidBody, Frame, State, Pose, Joint, JointType

# Optional OCC-dependent import — keeps pure data/math modules usable headless.
try:
    from .step_parser import StepParser
except ImportError:  # pragma: no cover
    StepParser = None  # type: ignore

__all__ = ['StepParser', 'RigidBody', 'Frame', 'State', 'Pose', 'Joint', 'JointType']
