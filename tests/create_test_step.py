"""
Create a test STEP file with explicit millimeter units
"""

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static

# Create a 100mm x 50mm x 30mm box
box = BRepPrimAPI_MakeBox(100, 50, 30).Shape()

# Create a STEP writer
writer = STEPControl_Writer()

# Set the unit to millimeters explicitly
Interface_Static.SetCVal("write.step.unit", "MM")
Interface_Static.SetCVal("xstep.cascade.unit", "MM")

print("Creating STEP file with millimeter units...")
print("Box dimensions: 100mm x 50mm x 30mm")
print("Expected volume: 150,000 mm³ = 0.00015 m³")

# Transfer the shape
writer.Transfer(box, STEPControl_AsIs)

# Write the file
status = writer.Write("tests/test_mm.step")

if status == 1:  # IFSelect_RetDone
    print("✓ Created tests/test_mm.step successfully")
else:
    print(f"✗ Failed to write STEP file. Status: {status}")
