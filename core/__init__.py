"""
Core functionality for Multi-Body Dynamics Preprocessor
"""

from .step_parser import StepParser
from .data_structures import RigidBody, Frame

__all__ = ['StepParser', 'RigidBody', 'Frame']
