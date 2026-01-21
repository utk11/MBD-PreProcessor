"""
Test with the original test.step file
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.step_parser import StepParser
from core.physics_calculator import PhysicsCalculator

print("\n" + "="*70)
print("Testing with test.step")
print("="*70)

# Load the STEP file
shape, unit_scale = StepParser.load_step_file("tests/test.step")

print(f"\n✓ Shape loaded successfully")

# Extract bodies
bodies = StepParser.extract_bodies_from_compound(shape)
print(f"✓ Found {len(bodies)} body/bodies\n")

# Calculate volumes with unit conversion
PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)

# Display summary
print("\n" + "="*70)
print("Summary:")
print("="*70)
for i, body in enumerate(bodies):
    print(f"{body.name}: {body.volume:.6e} m³")

print("\n")
