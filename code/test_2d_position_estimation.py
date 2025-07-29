#!/usr/bin/env python3
"""
Simple test script to verify the core 2D position estimation components
without requiring ROS or YOLO dependencies.
"""

import sys
import os
sys.path.append('/home/marius/DLML_TextToTurtleBot/code')

def test_core_components():
    print("Testing core 2D position estimation components...")
    
    try:
        from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator
        from classes.controllers.MapState import MapState
        from classes.topics.TFSubscriber import TFSubscriber
        
        print("✓ Core components imported successfully")
    except Exception as e:
        print(f"✗ Core component import failed: {e}")
        return False
    
    # Test MapState 2D functionality
    try:
        map_state = MapState()
        
        # Test 2D location storage
        map_state.add_location("test_location", (1.0, 2.0))
        location = map_state.get_location("test_location")
        assert location == (1.0, 2.0), f"Expected (1.0, 2.0), got {location}"
        
        # Test 2D object detection storage
        map_state.add_detected_object("chair", (3.0, 4.0), confidence=0.8)
        chairs = map_state.get_objects_by_class("chair")
        assert len(chairs) == 1, f"Expected 1 chair, got {len(chairs)}"
        assert chairs[0]['position'] == (3.0, 4.0), f"Position mismatch"
        assert chairs[0]['confidence'] == 0.8, f"Confidence mismatch"
        
        # Test 2D distance calculations
        nearest = map_state.get_nearest_object("chair", (0.0, 0.0))
        expected_distance = ((3.0 - 0.0) ** 2 + (4.0 - 0.0) ** 2) ** 0.5
        assert abs(nearest['distance'] - expected_distance) < 0.01, f"Distance calculation error"
        
        # Test radius search
        objects_in_radius = map_state.get_objects_in_radius((0.0, 0.0), 6.0)
        assert len(objects_in_radius) == 1, f"Should find 1 object within radius"
        
        objects_in_radius = map_state.get_objects_in_radius((0.0, 0.0), 4.0)
        assert len(objects_in_radius) == 0, f"Should find 0 objects within smaller radius"
        
        print("✓ MapState 2D functionality works correctly")
        
    except Exception as e:
        print(f"✗ MapState test failed: {e}")
        return False
    
    # Test ObjectPositionEstimator 2D functionality
    try:
        # Create estimator without TF subscriber for basic tests
        estimator = ObjectPositionEstimator(None)
        
        # Test bounding box center calculation
        bbox = {'x1': 100, 'y1': 100, 'x2': 200, 'y2': 200}
        center_x, center_y = estimator.get_object_center_pixel(bbox)
        assert center_x == 150 and center_y == 150, f"Center calculation failed"
        
        # Test pixel to angle conversion (2D - only horizontal)
        estimator.update_camera_params(640, 480)
        
        # Test center pixel (should be near zero angle)
        h_angle = estimator.pixel_to_angle(320, 240)
        assert abs(h_angle) < 0.01, f"Center pixel should have near-zero angle, got {h_angle}"
        
        # Test edge pixels
        left_angle = estimator.pixel_to_angle(0, 240)
        right_angle = estimator.pixel_to_angle(640, 240)
        assert left_angle < 0 and right_angle > 0, f"Left should be negative, right should be positive"
        
        # Test precision info with dimensions set
        precision_info = estimator.get_angular_precision_info()
        assert 'camera_horizontal_resolution_rad_per_pixel' in precision_info
        assert 'camera_horizontal_resolution_deg_per_pixel' in precision_info
        assert precision_info['camera_horizontal_resolution_rad_per_pixel'] is not None
        
        # Test precision info without dimensions
        estimator_no_dims = ObjectPositionEstimator(None)
        precision_info_no_dims = estimator_no_dims.get_angular_precision_info()
        assert 'error' in precision_info_no_dims
        assert precision_info_no_dims['camera_horizontal_resolution_rad_per_pixel'] is None
        
        # Test that pixel_to_angle raises error without dimensions
        try:
            estimator_no_dims.pixel_to_angle(320, 240)
            assert False, "Should have raised ValueError for missing camera dimensions"
        except ValueError as e:
            assert "Camera dimensions not set" in str(e)
        
        print("✓ ObjectPositionEstimator 2D functionality works correctly")
        
    except Exception as e:
        print(f"✗ ObjectPositionEstimator test failed: {e}")
        return False
    
    return True

def test_2d_position_calculation():
    print("\nTesting 2D position calculation logic...")
    
    try:
        import math
        from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator
        
        # Test the static quaternion to yaw conversion
        # Simple case: no rotation (identity quaternion)
        yaw = ObjectPositionEstimator.quaternion_to_yaw((0, 0, 0, 1))
        assert abs(yaw) < 0.01, f"Identity quaternion should give zero yaw"
        
        # Test 90 degree rotation around Z-axis
        # Quaternion for 90 degrees around Z: (0, 0, sin(π/4), cos(π/4))
        yaw = ObjectPositionEstimator.quaternion_to_yaw((0, 0, 0.7071, 0.7071))
        assert abs(yaw - math.pi/2) < 0.01, f"90 degree rotation failed"
        
        print("✓ 2D position calculation logic works correctly")
        
    except Exception as e:
        print(f"✗ 2D position calculation test failed: {e}")
        return False
    
    return True

def main():
    print("2D Object Position Estimation - Core Component Test")
    print("=" * 60)
    
    if not test_core_components():
        print("\n❌ Core component tests failed.")
        return 1
    
    if not test_2d_position_calculation():
        print("\n❌ 2D calculation tests failed.")
        return 1
    
    print("\n✅ All 2D position estimation tests passed!")
    print("\nThe system has been successfully simplified to work in 2D:")
    print("• Object positions are now stored as (x, y) coordinates")
    print("• Only horizontal camera angle is used for position estimation")
    print("• Distance calculations use 2D Euclidean distance")
    print("• All Z-coordinate handling has been removed")
    print("\nNext steps:")
    print("1. Run with ROS2 environment for full integration testing")
    print("2. Ensure camera and LIDAR topics are available")
    print("3. Check TF transforms for camera frames")
    print("4. Use demo_object_position_estimation.py for live testing")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
