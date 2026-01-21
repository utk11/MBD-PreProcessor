"""
Test loading the millimeter STEP file and verifying unit conversion
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.step_parser import StepParser
from core.physics_calculator import PhysicsCalculator
from core.data_structures import RigidBody

print("\n" + "="*60)
print("Testing Unit-Aware Volume Calculation")
print("="*60)

# Load the STEP file
print("\nLoading test_mm.step...")
shape, unit_scale = StepParser.load_step_file("tests/test_mm.step")

print(f"\n✓ Shape loaded successfully")
print(f"  Unit scale factor: {unit_scale} m/unit")

# Extract bodies
bodies = StepParser.extract_bodies_from_compound(shape)
print(f"\n✓ Found {len(bodies)} body/bodies")

# Calculate volumes with unit conversion
print(f"\nCalculating volumes with unit conversion:")
PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)

# Display results
print("\n" + "="*60)
print("Results:")
print("="*60)
for body in bodies:
    print(f"\n{body.name}:")
    print(f"  Volume: {body.volume:.6e} m³")
    print(f"  Volume: {body.volume * 1e6:.1f} mm³")

print("\nExpected for 100mm x 50mm x 30mm box:")
print(f"  Volume: 1.500000e-04 m³")
print(f"  Volume: 150000.0 mm³")

expected_volume_m3 = 0.00015
if len(bodies) > 0:
    actual_volume = bodies[0].volume
    error = abs(actual_volume - expected_volume_m3)
    print(f"\nError: {error:.6e} m³")
    if error < 1e-9:
        print("✓ PASSED: Volume calculation is correct!")
    else:
        print("✗ FAILED: Volume doesn't match expected value")

print("\n")
