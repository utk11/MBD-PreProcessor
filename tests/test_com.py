"""
Test script for Center of Mass calculation (Increment 8)
"""

from core.step_parser import StepParser
from core.physics_calculator import PhysicsCalculator

def test_com_calculation():
    """Test COM calculation on test STEP files"""
    
    test_files = [
        "test.step",
        "tests/test_mm.step"
    ]
    
    for filepath in test_files:
        try:
            print(f"\n{'='*60}")
            print(f"Testing: {filepath}")
            print('='*60)
            
            # Load STEP file
            shape, unit_scale = StepParser.load_step_file(filepath)
            
            if shape is None:
                print(f"Failed to load {filepath}")
                continue
            
            # Extract bodies
            bodies = StepParser.extract_bodies_from_compound(shape)
            print(f"Found {len(bodies)} bodies")
            
            # Calculate volumes
            PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)
            
            # Calculate centers of mass
            PhysicsCalculator.calculate_centers_of_mass_for_bodies(bodies, unit_scale)
            
            # Display results
            print(f"\n{'Results Summary':^60}")
            print('-'*60)
            for body in bodies:
                print(f"\n{body.name}:")
                print(f"  Volume: {body.volume:.6e} m³")
                if body.center_of_mass is not None:
                    com = body.center_of_mass
                    print(f"  COM: [{com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f}] m")
                    
                    # Verify COM is reasonable (not NaN or infinite)
                    if all(abs(c) < 1e10 for c in com):
                        print(f"  ✓ COM values are reasonable")
                    else:
                        print(f"  ✗ WARNING: COM values may be unreasonable")
                else:
                    print(f"  ✗ COM calculation failed")
            
            print('-'*60)
            
        except FileNotFoundError:
            print(f"File not found: {filepath}")
        except Exception as e:
            print(f"Error testing {filepath}: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    test_com_calculation()
    print(f"\n{'='*60}")
    print("COM calculation test complete!")
    print('='*60)
