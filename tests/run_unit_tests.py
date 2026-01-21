#!/usr/bin/env python
"""
Comprehensive test suite for unit-aware volume calculation
Run this to verify all unit conversions work correctly
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.step_parser import StepParser
from core.physics_calculator import PhysicsCalculator

def test_millimeter_file():
    """Test with millimeter STEP file"""
    print("\n" + "="*70)
    print("TEST 1: Millimeter STEP File (test_mm.step)")
    print("="*70)
    
    shape, unit_scale = StepParser.load_step_file("tests/test_mm.step")
    bodies = StepParser.extract_bodies_from_compound(shape)
    
    PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)
    
    # Expected: 100mm × 50mm × 30mm = 150,000 mm³ = 0.00015 m³
    expected = 0.00015
    actual = bodies[0].volume if bodies else 0
    error = abs(actual - expected)
    
    print(f"\nExpected: {expected:.6e} m³")
    print(f"Actual:   {actual:.6e} m³")
    print(f"Error:    {error:.6e} m³")
    
    if error < 1e-9:
        print("✓ TEST 1 PASSED")
        return True
    else:
        print("✗ TEST 1 FAILED")
        return False

def test_original_file():
    """Test with original test.step file"""
    print("\n" + "="*70)
    print("TEST 2: Original Assembly (test.step)")
    print("="*70)
    
    shape, unit_scale = StepParser.load_step_file("tests/test.step")
    bodies = StepParser.extract_bodies_from_compound(shape)
    
    PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)
    
    # Check that all volumes are reasonable (not absurdly large)
    all_reasonable = all(0 < body.volume < 1.0 for body in bodies)
    
    print(f"\nBodies found: {len(bodies)}")
    print(f"All volumes < 1 m³: {all_reasonable}")
    
    if len(bodies) > 0 and all_reasonable:
        print("✓ TEST 2 PASSED")
        return True
    else:
        print("✗ TEST 2 FAILED")
        return False

def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("UNIT-AWARE VOLUME CALCULATION TEST SUITE")
    print("="*70)
    
    results = []
    
    try:
        results.append(("Millimeter STEP", test_millimeter_file()))
    except Exception as e:
        print(f"✗ TEST 1 ERROR: {e}")
        results.append(("Millimeter STEP", False))
    
    try:
        results.append(("Original Assembly", test_original_file()))
    except Exception as e:
        print(f"✗ TEST 2 ERROR: {e}")
        results.append(("Original Assembly", False))
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    for test_name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        print(f"{test_name:30} {status}")
    
    all_passed = all(result[1] for result in results)
    
    print("\n" + "="*70)
    if all_passed:
        print("✓ ALL TESTS PASSED")
        print("Unit-aware volume calculation is working correctly!")
    else:
        print("✗ SOME TESTS FAILED")
        print("Please review the errors above")
    print("="*70 + "\n")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    exit(main())
