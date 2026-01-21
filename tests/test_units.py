"""
Test to extract unit information from STEP files
"""

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone

# Create a reader
reader = STEPControl_Reader()

# Read the test STEP file
status = reader.ReadFile("tests/test.step")

if status == IFSelect_RetDone:
    print("✓ STEP file read successfully")
    
    # Transfer the file content
    reader.TransferRoots()
    
    # Get the system length unit
    length_unit = reader.SystemLengthUnit()
    print(f"\nSystem Length Unit: {length_unit} meters")
    print(f"This means 1 unit in the model = {length_unit} meters")
    
    if length_unit == 0.001:
        print("→ Model is in MILLIMETERS (mm)")
        print(f"→ To convert volume: multiply by {length_unit**3} = {length_unit**3}")
    elif length_unit == 0.01:
        print("→ Model is in CENTIMETERS (cm)")
        print(f"→ To convert volume: multiply by {length_unit**3} = {length_unit**3}")
    elif length_unit == 1.0:
        print("→ Model is in METERS (m)")
        print(f"→ Volume is already in m³")
    elif length_unit == 0.0254:
        print("→ Model is in INCHES (in)")
        print(f"→ To convert volume: multiply by {length_unit**3} = {length_unit**3}")
    else:
        print(f"→ Unknown unit system")
    
    # Try to get file units
    from OCC.Core.TColStd import TColStd_SequenceOfAsciiString
    length_names = TColStd_SequenceOfAsciiString()
    angle_names = TColStd_SequenceOfAsciiString()
    solid_angle_names = TColStd_SequenceOfAsciiString()
    
    reader.FileUnits(length_names, angle_names, solid_angle_names)
    
    print(f"\nFile Unit Names:")
    print(f"  Length units found: {length_names.Size()}")
    for i in range(1, length_names.Size() + 1):
        print(f"    - {length_names.Value(i).ToCString()}")
    
    print(f"  Angle units found: {angle_names.Size()}")
    for i in range(1, angle_names.Size() + 1):
        print(f"    - {angle_names.Value(i).ToCString()}")
    
else:
    print(f"✗ Failed to read STEP file. Status: {status}")
