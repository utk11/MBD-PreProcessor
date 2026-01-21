"""
Test script for Increment 14: Face Selection Mode
Tests geometry_utils functionality without GUI
"""

from core.step_parser import StepParser
from core.geometry_utils import GeometryUtils
import os

# Load a STEP file
step_file = "test.step"

if not os.path.exists(step_file):
    print(f"Error: {step_file} not found")
    exit(1)

print(f"Loading {step_file}...")
shape, unit_scale = StepParser.load_step_file(step_file)

if shape is None:
    print("Error: Could not load STEP file")
    exit(1)

print(f"STEP file loaded successfully")
print(f"Unit scale: {unit_scale}")

# Extract bodies
bodies = StepParser.extract_bodies_from_compound(shape)
print(f"\nFound {len(bodies)} bodies")

# Test face extraction for each body
print("\n=== Testing Face Extraction ===")
for body in bodies[:3]:  # Test first 3 bodies
    print(f"\nBody {body.id} ({body.name}):")
    faces = GeometryUtils.extract_faces(body.shape)
    print(f"  Faces: {len(faces)}")
    
    if len(faces) > 0:
        for i, face_props in enumerate(faces[:3]):  # Show first 3 faces
            print(f"    Face {i}:")
            print(f"      Area: {face_props.area:.6e} m²")
            print(f"      Center: [{face_props.center[0]:.6f}, {face_props.center[1]:.6f}, {face_props.center[2]:.6f}] m")
            print(f"      Normal: [{face_props.normal[0]:.6f}, {face_props.normal[1]:.6f}, {face_props.normal[2]:.6f}]")
        if len(faces) > 3:
            print(f"    ... and {len(faces) - 3} more faces")

# Test edge extraction for each body
print("\n\n=== Testing Edge Extraction ===")
for body in bodies[:3]:  # Test first 3 bodies
    print(f"\nBody {body.id} ({body.name}):")
    edges = GeometryUtils.extract_edges(body.shape)
    print(f"  Edges: {len(edges)}")
    
    if len(edges) > 0:
        for i, edge_props in enumerate(edges[:3]):  # Show first 3 edges
            print(f"    Edge {i}:")
            print(f"      Length: {edge_props.length:.6f} m")
            print(f"      Midpoint: [{edge_props.midpoint[0]:.6f}, {edge_props.midpoint[1]:.6f}, {edge_props.midpoint[2]:.6f}] m")
            print(f"      Direction: [{edge_props.direction[0]:.6f}, {edge_props.direction[1]:.6f}, {edge_props.direction[2]:.6f}]")
        if len(edges) > 3:
            print(f"    ... and {len(edges) - 3} more edges")

print("\n\n✓ Increment 14 geometry extraction test complete!")
