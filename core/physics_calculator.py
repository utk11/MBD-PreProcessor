"""
Physics calculations for Multi-Body Dynamics Preprocessor
Handles volume, center of mass, and inertia tensor calculations
"""

from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.TopoDS import TopoDS_Shape
from typing import Optional, List
import numpy as np


class PhysicsCalculator:
    """Calculate physical properties of bodies"""
    
    @staticmethod
    def calculate_volume(shape: TopoDS_Shape) -> Optional[float]:
        """
        Calculate volume of a solid body using OCC GProp
        
        Args:
            shape: The TopoDS_Shape to calculate volume for
            
        Returns:
            Volume in cubic meters (m³), or None if calculation fails
        """
        try:
            # Create a GProp object to store geometric properties
            props = GProp_GProps()
            
            # Calculate volume properties using the new static method API
            brepgprop.VolumeProperties(shape, props)
            
            # Get the volume (in m³ if the model units are meters)
            volume = props.Mass()
            
            # Verify volume is positive (valid solid)
            if volume <= 0:
                print(f"Warning: Calculated volume is non-positive: {volume}")
                return None
            
            return volume
            
        except Exception as e:
            print(f"Error calculating volume: {e}")
            return None
    
    @staticmethod
    def calculate_volumes_for_bodies(bodies: list, unit_scale: float = 1.0) -> None:
        """
        Calculate and store volumes for a list of bodies
        
        Args:
            bodies: List of RigidBody objects
            unit_scale: Scale factor to convert model units to meters (e.g., 0.001 for mm)
                       Volume will be scaled by unit_scale³ to get m³
        """
        # Calculate the volume scale factor (cube of linear scale)
        volume_scale = unit_scale ** 3
        
        print(f"Unit scale: {unit_scale} m/unit")
        print(f"Volume scale: {volume_scale} (model_units³ to m³)")
        
        for body in bodies:
            volume = PhysicsCalculator.calculate_volume(body.shape)
            if volume is not None:
                # Convert volume to m³ using the scale factor
                volume_m3 = volume * volume_scale
                body.volume = volume_m3
                print(f"Body {body.id} ({body.name}): Volume = {volume:.6e} model³ = {volume_m3:.6e} m³")
            else:
                body.volume = 0.0
                print(f"Body {body.id} ({body.name}): Volume calculation failed")
    
    @staticmethod
    def calculate_center_of_mass(shape: TopoDS_Shape, unit_scale: float = 1.0) -> Optional[List[float]]:
        """
        Calculate center of mass of a solid body using OCC GProp
        
        Args:
            shape: The TopoDS_Shape to calculate COM for
            unit_scale: Scale factor to convert model units to meters (e.g., 0.001 for mm)
            
        Returns:
            Center of mass as [x, y, z] in meters, or None if calculation fails
        """
        try:
            # Create a GProp object to store geometric properties
            props = GProp_GProps()
            
            # Calculate volume properties (needed for COM)
            brepgprop.VolumeProperties(shape, props)
            
            # Get the center of gravity (center of mass for uniform density)
            com_point = props.CentreOfMass()
            
            # Convert to [x, y, z] list and scale to meters
            com = [
                com_point.X() * unit_scale,
                com_point.Y() * unit_scale,
                com_point.Z() * unit_scale
            ]
            
            return com
            
        except Exception as e:
            print(f"Error calculating center of mass: {e}")
            return None
    
    @staticmethod
    def calculate_centers_of_mass_for_bodies(bodies: list, unit_scale: float = 1.0) -> None:
        """
        Calculate and store center of mass for a list of bodies
        
        Args:
            bodies: List of RigidBody objects
            unit_scale: Scale factor to convert model units to meters (e.g., 0.001 for mm)
        """
        print(f"\nCalculating centers of mass...")
        print(f"Unit scale: {unit_scale} m/unit")
        
        for body in bodies:
            com = PhysicsCalculator.calculate_center_of_mass(body.shape, unit_scale)
            if com is not None:
                body.center_of_mass = com
                print(f"Body {body.id} ({body.name}): COM = [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}] m")
            else:
                body.center_of_mass = None
                print(f"Body {body.id} ({body.name}): COM calculation failed")
        
        print("Center of mass calculation complete.")
    
    @staticmethod
    def calculate_inertia_tensor(shape: TopoDS_Shape, unit_scale: float = 1.0) -> Optional[np.ndarray]:
        """
        Calculate inertia tensor of a solid body using OCC GProp
        Normalized about the center of mass with unit density
        
        Args:
            shape: The TopoDS_Shape to calculate inertia for
            unit_scale: Scale factor to convert model units to meters (e.g., 0.001 for mm)
            
        Returns:
            3×3 inertia tensor matrix in kg·m² (assuming unit density), or None if calculation fails
        """
        try:
            # Create a GProp object to store geometric properties
            props = GProp_GProps()
            
            # Calculate volume properties (includes inertia)
            brepgprop.VolumeProperties(shape, props)
            
            # Get the inertia matrix at the center of mass
            # The matrix is stored in model units, need to scale appropriately
            # Inertia scales as (length^5) for geometric inertia
            # For mass inertia: I = ρ * geometric_inertia
            # We use unit density (ρ = 1 kg/m³) for normalized values
            
            # Get center of mass
            com = props.CentreOfMass()
            
            # Get inertia matrix at the center of mass
            # Note: props.MatrixOfInertia() returns the central inertia tensor (about COM) for the shape
            matrix_com = props.MatrixOfInertia()
            
            # Extract inertia components
            Ixx = matrix_com.Value(1, 1)
            Iyy = matrix_com.Value(2, 2)
            Izz = matrix_com.Value(3, 3)
            Ixy = matrix_com.Value(1, 2)
            Ixz = matrix_com.Value(1, 3)
            Iyz = matrix_com.Value(2, 3)
            
            # Scale to SI units (kg·m²)
            # Geometric inertia scales as length^5
            # Mass inertia = density * geometric_inertia
            # For unit density (1 kg/m³): I_SI = I_model * unit_scale^5
            inertia_scale = unit_scale ** 5
            
            # Create symmetric 3×3 inertia tensor
            # OCC returns the tensor components, so we use them directly
            inertia_tensor = np.array([
                [Ixx * inertia_scale, Ixy * inertia_scale, Ixz * inertia_scale],
                [Ixy * inertia_scale, Iyy * inertia_scale, Iyz * inertia_scale],
                [Ixz * inertia_scale, Iyz * inertia_scale, Izz * inertia_scale]
            ])
            
            return inertia_tensor
            
        except Exception as e:
            print(f"Error calculating inertia tensor: {e}")
            return None
    
    @staticmethod
    def calculate_inertia_tensors_for_bodies(bodies: list, unit_scale: float = 1.0) -> None:
        """
        Calculate and store inertia tensors for a list of bodies
        
        Args:
            bodies: List of RigidBody objects
            unit_scale: Scale factor to convert model units to meters (e.g., 0.001 for mm)
        """
        print(f"\nCalculating inertia tensors...")
        print(f"Unit scale: {unit_scale} m/unit")
        print(f"Inertia scale: {unit_scale**5} (model_units^5 to kg·m²)")
        
        for body in bodies:
            inertia = PhysicsCalculator.calculate_inertia_tensor(body.shape, unit_scale)
            if inertia is not None:
                body.inertia_tensor = inertia
                print(f"Body {body.id} ({body.name}): Inertia tensor calculated")
                # Print diagonal elements for verification
                print(f"  Diagonal: [{inertia[0,0]:.6e}, {inertia[1,1]:.6e}, {inertia[2,2]:.6e}] kg·m²")
            else:
                body.inertia_tensor = None
                print(f"Body {body.id} ({body.name}): Inertia calculation failed")
        
        print("Inertia tensor calculation complete.")    
    @staticmethod
    def initialize_local_frames(bodies: list):
        """
        Initialize local coordinate frames for all bodies at their center of mass.
        Frames are initialized with identity rotation (aligned with world axes).
        
        Args:
            bodies: List of RigidBody objects with calculated COM
        """
        print("Initializing local frames...")
        
        from core.data_structures import Frame
        
        for body in bodies:
            if body.center_of_mass is not None:
                # Create local frame at COM with identity rotation
                body.local_frame = Frame(
                    origin=body.center_of_mass.copy(),
                    rotation_matrix=np.eye(3),
                    name=f"{body.name}_LocalFrame"
                )
                print(f"Body {body.id} ({body.name}): Local frame created at COM {body.center_of_mass}")
            else:
                print(f"Warning: Body {body.id} ({body.name}): Cannot create local frame - COM not calculated")
        
        print("Local frame initialization complete.\n")