"""
STEP file parsing and loading utilities
"""

from typing import Optional, List, Tuple
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Solid
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from .data_structures import RigidBody


class StepParser:
    """Handles loading and parsing of STEP files"""
    
    @staticmethod
    def load_step_file(filepath: str) -> Tuple[Optional[TopoDS_Shape], float]:
        """
        Load a STEP file and return the shape with unit information
        
        Args:
            filepath: Path to the STEP file (.step or .stp)
            
        Returns:
            Tuple of (TopoDS_Shape, unit_scale_factor)
            - TopoDS_Shape if successful, None if failed
            - unit_scale_factor: meters per model unit (e.g., 0.001 for mm)
            
        Raises:
            FileNotFoundError: If the file doesn't exist
            Exception: For other loading errors
        """
        try:
            from OCC.Core.TColStd import TColStd_SequenceOfAsciiString
            
            # Create STEP reader
            reader = STEPControl_Reader()
            
            # Read the STEP file
            status = reader.ReadFile(filepath)
            
            if status != IFSelect_RetDone:
                raise Exception(f"Error reading STEP file: status code {status}")
            
            # Get file unit declarations BEFORE transfer
            # This tells us what units the file actually uses
            length_names = TColStd_SequenceOfAsciiString()
            angle_names = TColStd_SequenceOfAsciiString()
            solid_angle_names = TColStd_SequenceOfAsciiString()
            
            reader.FileUnits(length_names, angle_names, solid_angle_names)
            
            # Extract the unit name from the file
            file_unit_name = "UNKNOWN"
            if length_names.Size() > 0:
                file_unit_name = length_names.Value(1).ToCString().upper()
            
            # Map file unit names to conversion factors (to meters)
            unit_map = {
                "METRE": 1.0,
                "METER": 1.0,
                "M": 1.0,
                "MILLIMETRE": 0.001,
                "MILLIMETER": 0.001,
                "MM": 0.001,
                "CENTIMETRE": 0.01,
                "CENTIMETER": 0.01,
                "CM": 0.01,
                "INCH": 0.0254,
                "IN": 0.0254,
                "FOOT": 0.3048,
                "FT": 0.3048,
            }
            
            # Determine the unit scale factor
            unit_scale = unit_map.get(file_unit_name, 1.0)
            
            # Transfer the roots to get the shape
            reader.TransferRoots()
            shape = reader.OneShape()
            
            # Validate that the shape is not null/empty
            if shape.IsNull():
                raise Exception("STEP file is empty or invalid - no solid shapes found")

            # Determine unit name for display
            unit_name = "unknown"
            if abs(unit_scale - 1.0) < 1e-9:
                unit_name = "meters"
            elif abs(unit_scale - 0.001) < 1e-9:
                unit_name = "millimeters"
            elif abs(unit_scale - 0.01) < 1e-9:
                unit_name = "centimeters"
            elif abs(unit_scale - 0.0254) < 1e-9:
                unit_name = "inches"
            
            print(f"STEP file declares units: {file_unit_name}")
            print(f"Using unit scale: {unit_scale} m/unit ({unit_name})")
            print(f"Volume scale factor: {unit_scale**3} (to convert to m³)")
            
            return shape, unit_scale
            
        except FileNotFoundError:
            raise FileNotFoundError(f"STEP file not found: {filepath}")
        except Exception as e:
            # Print full traceback for debugging purposes
            import traceback
            traceback.print_exc()
            raise Exception(f"Error reading STEP file: {str(e)}")
    
    @staticmethod
    def extract_bodies_from_compound(shape: TopoDS_Shape) -> List[RigidBody]:
        """
        Extract individual solid bodies from a compound shape
        
        Args:
            shape: The TopoDS_Shape (can be compound or single solid)
            
        Returns:
            List of RigidBody objects, each representing a solid body
        """
        bodies = []
        body_id = 0
        
        # Use TopExp_Explorer to iterate through all solids in the shape
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        
        while explorer.More():
            solid = explorer.Current()
            # Create a RigidBody for each solid found
            body = RigidBody(body_id, solid)
            bodies.append(body)
            body_id += 1
            explorer.Next()
        
        # If no solids found but shape is valid, treat entire shape as one body
        if len(bodies) == 0 and not shape.IsNull():
            body = RigidBody(0, shape, "Body_0")
            bodies.append(body)
        
        return bodies
    
    @staticmethod
    def validate_step_file(filepath: str) -> bool:
        """
        Check if a file is a valid STEP file
        
        Args:
            filepath: Path to check
            
        Returns:
            True if file appears to be a valid STEP file
        """
        valid_extensions = ['.step', '.stp', '.STEP', '.STP']
        return any(filepath.endswith(ext) for ext in valid_extensions)
