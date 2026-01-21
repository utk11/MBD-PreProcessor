import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.data_structures import Joint, JointType, Frame, Assembly, RigidBody
import numpy as np

def test_joint_creation():
    print("Testing Joint Data Structure...")

    # 1. Create an Assembly
    assembly = Assembly()
    print("Assembly created.")

    # 2. Mock some bodies (don't need actual shapes for this test)
    # We pass None for shape as we are testing data structure
    body1 = RigidBody(0, None, "Ground")
    body2 = RigidBody(1, None, "Link1")
    
    assembly.bodies[0] = body1
    assembly.bodies[1] = body2
    print(f"Added bodies: {body1.name}, {body2.name}")

    # 3. Create Frames for the joint
    # Frame on Body 1
    f1 = Frame(origin=[0, 0, 0], name="J1_Frame1")
    # Frame on Body 2 (e.g., at local [0, -1, 0] if connected)
    f2 = Frame(origin=[0, 0.5, 0], name="J1_Frame2")

    # 4. Create a Revolution Joint
    joint_name = "Joint_1"
    joint = Joint(
        name=joint_name,
        joint_type=JointType.REVOLUTE,
        body1_id=0,
        body2_id=1,
        frame1=f1,
        frame2=f2
    )

    # 5. Add to Assembly
    assembly.joints[joint_name] = joint
    print(f"Created joint: {joint}")

    # 6. Verify assertions
    assert joint.name == "Joint_1"
    assert joint.joint_type == JointType.REVOLUTE
    assert joint.body1_id == 0
    assert joint.body2_id == 1
    assert joint.frame1 == f1
    assert joint.frame2 == f2
    
    assert "Joint_1" in assembly.joints
    retrieved_joint = assembly.joints["Joint_1"]
    assert retrieved_joint == joint

    print("Joint creation and storage verification successful!")

if __name__ == "__main__":
    test_joint_creation()
