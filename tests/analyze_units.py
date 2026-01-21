"""
Detailed investigation of STEP file unit handling
"""

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TColStd import TColStd_SequenceOfAsciiString

def analyze_step_file(filepath):
    print(f"\n{'='*70}")
    print(f"Analyzing: {filepath}")
    print('='*70)
    
    reader = STEPControl_Reader()
    status = reader.ReadFile(filepath)
    
    if status != IFSelect_RetDone:
        print("Failed to read file")
        return
    
    # Get file units (what the file declares)
    length_names = TColStd_SequenceOfAsciiString()
    angle_names = TColStd_SequenceOfAsciiString()
    solid_angle_names = TColStd_SequenceOfAsciiString()
    
    reader.FileUnits(length_names, angle_names, solid_angle_names)
    
    print(f"\nFile Unit Declarations:")
    print(f"  Length units in file: ", end="")
    for i in range(1, length_names.Size() + 1):
        unit_name = length_names.Value(i).ToCString()
        print(f"{unit_name}", end=" ")
    print()
    
    # Transfer and get system unit
    reader.TransferRoots()
    system_unit = reader.SystemLengthUnit()
    
    print(f"\nSystem Unit After Transfer:")
    print(f"  SystemLengthUnit(): {system_unit} meters/model_unit")
    
    # Explanation
    print(f"\nInterpretation:")
    print(f"  - File declares units as: {length_names.Value(1).ToCString() if length_names.Size() > 0 else 'unknown'}")
    print(f"  - PyOCC converts to: {system_unit} meters per model unit")
    
    if abs(system_unit - 1.0) < 1e-9:
        print(f"  - Geometry is in METERS internally")
        print(f"  - 1 model unit = 1 meter")
    elif abs(system_unit - 0.001) < 1e-9:
        print(f"  - Geometry is in MILLIMETERS internally")  
        print(f"  - 1 model unit = 0.001 meters")
    
    # Get the shape to check actual coordinates
    shape = reader.OneShape()
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    bbox = Bnd_Box()
    brepbndlib.Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    
    print(f"\nBounding Box (in model coordinates):")
    print(f"  X: {xmin:.6f} to {xmax:.6f} (size: {xmax-xmin:.6f})")
    print(f"  Y: {ymin:.6f} to {ymax:.6f} (size: {ymax-ymin:.6f})")
    print(f"  Z: {zmin:.6f} to {zmax:.6f} (size: {zmax-zmin:.6f})")
    
    print(f"\nIn physical units (meters):")
    print(f"  X: {xmin*system_unit:.6f} to {xmax*system_unit:.6f} m (size: {(xmax-xmin)*system_unit:.6f} m)")
    print(f"  Y: {ymin*system_unit:.6f} to {ymax*system_unit:.6f} m (size: {(ymax-ymin)*system_unit:.6f} m)")
    print(f"  Z: {zmin*system_unit:.6f} to {zmax*system_unit:.6f} m (size: {(zmax-zmin)*system_unit:.6f} m)")

# Analyze both files
analyze_step_file("tests/test.step")
analyze_step_file("tests/test_mm.step")

print(f"\n{'='*70}")
print("CONCLUSION:")
print('='*70)
print("PyOCC's STEPControl_Reader converts geometry to a normalized coordinate")
print("system during import. The SystemLengthUnit() tells us the conversion factor.")
print("However, it appears to always report 1.0 (meters) after transfer.")
print("\nWe should use the FILE units and coordinate magnitudes to determine scaling.")
print('='*70)
print()
