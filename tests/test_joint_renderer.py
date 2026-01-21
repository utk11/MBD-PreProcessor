import sys
import os
import unittest
from unittest.mock import MagicMock
import numpy as np

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.data_structures import Joint, JointType, Frame
from visualization.joint_renderer import JointRenderer

class TestJointRenderer(unittest.TestCase):
    def setUp(self):
        # Mock the OCC display
        self.mock_display = MagicMock()
        self.mock_display.Context = MagicMock()
        
        # Initialize renderer
        self.renderer = JointRenderer(self.mock_display)
        # Mock the internal frame renderer to avoid complex dependency
        self.renderer.frame_renderer = MagicMock()
        self.renderer.frame_renderer.unit_scale = 1.0

    def test_render_joint(self):
        # Create a test joint
        f1 = Frame(origin=[0, 0, 0], name="F1")
        f2 = Frame(origin=[1, 1, 1], name="F2")
        joint = Joint("TestJoint", JointType.REVOLUTE, 0, 1, f1, f2)
        
        # Call render
        self.renderer.render_joint(joint)
        
        # Verify frame renderer was called twice
        self.assertEqual(self.renderer.frame_renderer.render_frame.call_count, 2)
        
        # Verify line was added to display
        # We expect Context.Display to be called for the line
        self.mock_display.Context.Display.assert_called()
        
        # Verify joint objects are tracked
        self.assertIn("TestJoint", self.renderer.joint_objects)
        objects = self.renderer.joint_objects["TestJoint"]
        self.assertEqual(len(objects), 3) # 2 frame names (str) + 1 AIS_Line object
        
    def test_remove_joint(self):
        # Setup pre-existing joint
        self.renderer.joint_objects["TestJoint"] = ["FrameName1", "FrameName2", MagicMock()]
        
        # Call remove
        self.renderer.remove_joint("TestJoint")
        
        # Verify frame renderer remove was called
        self.renderer.frame_renderer.remove_frame.assert_any_call("FrameName1")
        self.renderer.frame_renderer.remove_frame.assert_any_call("FrameName2")
        
        # Verify display remove was called
        self.mock_display.Context.Remove.assert_called()
        
        # Verify removed from tracking
        self.assertNotIn("TestJoint", self.renderer.joint_objects)

if __name__ == '__main__':
    unittest.main()
