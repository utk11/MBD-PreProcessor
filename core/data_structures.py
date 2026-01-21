"""
Data structures for Multi-Body Dynamics Preprocessor
"""

from typing import Optional, Dict, List
import numpy as np
from enum import Enum, auto
from OCC.Core.TopoDS import TopoDS_Shape


class JointType(Enum):
    """Enumeration of supported joint types"""
    FIXED = auto()
    REVOLUTE = auto()
    PRISMATIC = auto()
    CYLINDRICAL = auto()
    SPHERICAL = auto()


class MotorType(Enum):
    """Enumeration of motor actuation types"""
    VELOCITY = auto()    # Speed control (rad/s for revolute, m/s for prismatic)
    TORQUE = auto()      # Torque/Force control (N·m for revolute, N for prismatic)
    POSITION = auto()    # Position control (rad for revolute, m for prismatic)


class Force:
    """Represents an external force applied to a body"""
    
    def __init__(self, name: str, body_id: int, frame: 'Frame', 
                 magnitude: float, direction: np.ndarray):
        """
        Initialize a force applied to a body
        
        Args:
            name: Unique name for the force
            body_id: ID of the body the force acts on
            frame: Application point and reference frame
            magnitude: Magnitude of the force in Newtons
            direction: Direction vector [x, y, z] (will be normalized)
        """
        self.name = name
        self.body_id = body_id
        self.frame = frame
        self.magnitude = abs(float(magnitude))
        
        # Normalize direction vector
        dir_array = np.array(direction, dtype=float)
        norm = np.linalg.norm(dir_array)
        if norm < 1e-10:
            raise ValueError("Direction vector cannot be zero")
        self.direction = dir_array / norm
    
    def get_force_vector(self) -> np.ndarray:
        """Get the force vector (magnitude * direction)"""
        return self.magnitude * self.direction
    
    def __repr__(self):
        return f"Force(name='{self.name}', body={self.body_id}, magnitude={self.magnitude}N)"


class Torque:
    """Represents an external torque (moment) applied to a body"""
    
    def __init__(self, name: str, body_id: int, frame: 'Frame', 
                 magnitude: float, axis: np.ndarray):
        """
        Initialize a torque applied to a body
        
        Args:
            name: Unique name for the torque
            body_id: ID of the body the torque acts on
            frame: Application point and reference frame
            magnitude: Magnitude of the torque in Newton-meters
            axis: Rotation axis vector [x, y, z] (will be normalized)
        """
        self.name = name
        self.body_id = body_id
        self.frame = frame
        self.magnitude = abs(float(magnitude))
        
        # Normalize axis vector
        axis_array = np.array(axis, dtype=float)
        norm = np.linalg.norm(axis_array)
        if norm < 1e-10:
            raise ValueError("Axis vector cannot be zero")
        self.axis = axis_array / norm
    
    def get_torque_vector(self) -> np.ndarray:
        """Get the torque vector (magnitude * axis)"""
        return self.magnitude * self.axis
    
    def __repr__(self):
        return f"Torque(name='{self.name}', body={self.body_id}, magnitude={self.magnitude}N·m)"


class Joint:
    """Represents a kinematic joint between two bodies"""
    
    def __init__(self, name: str, joint_type: JointType, 
                 body1_id: int, body2_id: int, 
                 frame: 'Frame' = None,
                 axis: str = "+Z"):
        """
        Initialize a joint between two bodies
        
        Args:
            name: Unique name for the joint
            joint_type: Type of joint (Fixed, Revolute, etc.)
            body1_id: ID of the first body (-1 for ground)
            body2_id: ID of the second body
            frame: Joint frame in global world coordinates
            axis: Axis of rotation/translation relative to the joint frame ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
            
        Note: Frame is stored in world coordinates since this is a preprocessor 
              where bodies don't move. No local-to-global conversion is needed.
        """
        self.name = name
        self.joint_type = joint_type
        self.body1_id = body1_id
        self.body2_id = body2_id
        self.frame = frame if frame is not None else Frame(name=f"{name}_Frame")
        self.axis = axis
        
        # Motor properties (add-on for revolute and prismatic joints)
        self.is_motorized: bool = False
        self.motor_type: Optional[MotorType] = None
        self.motor_value: float = 0.0
    
    def add_motor(self, motor_type: MotorType, value: float):
        """
        Add motor actuation to this joint
        
        Args:
            motor_type: Type of motor (VELOCITY, TORQUE, POSITION)
            value: Motor value (rad/s or m/s for VELOCITY, N·m or N for TORQUE, rad or m for POSITION)
            
        Raises:
            ValueError: If joint type doesn't support motors or if already motorized
        """
        if self.joint_type not in [JointType.REVOLUTE, JointType.PRISMATIC]:
            raise ValueError(f"Motors only supported on REVOLUTE and PRISMATIC joints, not {self.joint_type.name}")
        
        if self.is_motorized:
            raise ValueError(f"Joint '{self.name}' already has a motor. Remove it first.")
        
        self.is_motorized = True
        self.motor_type = motor_type
        self.motor_value = float(value)
    
    def remove_motor(self):
        """Remove motor actuation from this joint"""
        self.is_motorized = False
        self.motor_type = None
        self.motor_value = 0.0
    
    def get_motor_description(self) -> str:
        """Get a human-readable description of the motor"""
        if not self.is_motorized:
            return "No motor"
        
        unit = ""
        if self.motor_type == MotorType.VELOCITY:
            unit = "rad/s" if self.joint_type == JointType.REVOLUTE else "m/s"
        elif self.motor_type == MotorType.TORQUE:
            unit = "N·m" if self.joint_type == JointType.REVOLUTE else "N"
        elif self.motor_type == MotorType.POSITION:
            unit = "rad" if self.joint_type == JointType.REVOLUTE else "m"
        
        return f"{self.motor_type.name}: {self.motor_value} {unit}"

    def __repr__(self):
        motor_str = f", motorized={self.is_motorized}" if self.is_motorized else ""
        return f"Joint(name='{self.name}', type={self.joint_type.name}, axis='{self.axis}', bodies=({self.body1_id}, {self.body2_id}){motor_str})"


class Assembly:
    """
    Container for the multi-body system.
    Holds all bodies, joints, and user-defined frames.
    """
    def __init__(self):
        self.bodies: Dict[int, RigidBody] = {}
        self.joints: Dict[str, Joint] = {}
        self.frames: Dict[str, Frame] = {}  # User-defined frames


class Frame:
    """Represents a coordinate frame with origin and orientation"""
    
    def __init__(self, origin: np.ndarray = None, rotation_matrix: np.ndarray = None, name: str = "Frame"):
        """
        Initialize a coordinate frame
        
        Args:
            origin: 3D position [x, y, z] in meters (default: [0, 0, 0])
            rotation_matrix: 3×3 rotation matrix (default: identity matrix)
            name: Optional name for the frame
        """
        self.name = name
        
        # Set origin (default to world origin)
        if origin is None:
            self.origin = np.array([0.0, 0.0, 0.0])
        else:
            self.origin = np.array(origin)
        
        # Set rotation matrix (default to identity)
        if rotation_matrix is None:
            self.rotation_matrix = np.eye(3)
        else:
            self.rotation_matrix = np.array(rotation_matrix)
    
    def get_x_axis(self) -> np.ndarray:
        """Get the X-axis direction (first column of rotation matrix)"""
        return self.rotation_matrix[:, 0]
    
    def get_y_axis(self) -> np.ndarray:
        """Get the Y-axis direction (second column of rotation matrix)"""
        return self.rotation_matrix[:, 1]
    
    def get_z_axis(self) -> np.ndarray:
        """Get the Z-axis direction (third column of rotation matrix)"""
        return self.rotation_matrix[:, 2]
    
    def get_euler_angles(self) -> np.ndarray:
        """
        Get Euler angles (Extrinsic XYZ convention) in degrees
        Returns: [x_angle, y_angle, z_angle]
        """
        R = self.rotation_matrix
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.degrees(np.array([x, y, z]))

    def set_rotation_from_euler(self, angles_deg: np.ndarray):
        """
        Set rotation matrix from Euler angles (Extrinsic XYZ convention) in degrees
        Args:
            angles_deg: [x_angle, y_angle, z_angle]
        """
        angles_rad = np.radians(angles_deg)
        x, y, z = angles_rad

        cx, sx = np.cos(x), np.sin(x)
        cy, sy = np.cos(y), np.sin(y)
        cz, sz = np.cos(z), np.sin(z)

        # Rx
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        # Ry
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        # Rz
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        # R = Rz * Ry * Rx (Extrinsic XYZ)
        self.rotation_matrix = Rz @ Ry @ Rx

    def __repr__(self):
        return f"Frame(name='{self.name}', origin={self.origin})"


class RigidBody:
    """Represents a rigid body in the assembly"""
    
    def __init__(self, body_id: int, shape: TopoDS_Shape, name: Optional[str] = None):
        """
        Initialize a rigid body
        
        Args:
            body_id: Unique identifier for the body
            shape: The TopoDS_Shape representing the body geometry
            name: Optional name for the body (default: Body_{id})
        """
        self.id = body_id
        self.shape = shape
        self.name = name if name is not None else f"Body_{body_id}"
        
        # Physical properties (calculated later)
        self.volume = 0.0  # Volume in m³
        self.center_of_mass = None  # Center of mass [x, y, z] in meters
        self.inertia_tensor = None  # Inertia tensor 3×3 matrix in kg·m² (normalized about COM)
        
        # Local coordinate frame (initialized after COM calculation)
        self.local_frame: Optional[Frame] = None  # Frame at COM with identity rotation

        # Visual property
        self.visible = True
        
        # Contact detection property
        self.contact_enabled = True  # Enable contact detection by default
        
    def __repr__(self):
        return f"RigidBody(id={self.id}, name='{self.name}')"
