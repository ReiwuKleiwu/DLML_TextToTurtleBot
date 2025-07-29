#!/usr/bin/env python3
"""
Quick test script to verify that all the object position estimation components
can be imported and instantiated correctly.
"""

import sys
sys.path.append('/home/marius/DLML_TextToTurtleBot/code')

def test_imports():
    print("Testing imports...")
    
    try:
        from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator
        print("✓ ObjectPositionEstimator imported successfully")
    except Exception as e:
        print(f"✗ ObjectPositionEstimator import failed: {e}")
        return False
    
    try:
        from classes.controllers.MapState import MapState
        print("✓ MapState imported successfully")
    except Exception as e:
        print(f"✗ MapState import failed: {e}")
        return False
    
    try:
        from classes.sensors.CameraHandler import CameraHandler
        print("✓ CameraHandler imported successfully")
    except Exception as e:
        print(f"✗ CameraHandler import failed: {e}")
        return False
    
    try:
        from classes.sensors.LIDARHandler import LIDARHandler
        print("✓ LIDARHandler imported successfully")
    except Exception as e:
        print(f"✗ LIDARHandler import failed: {e}")
        return False
    
    try:
        from classes.topics.TFSubscriber import TFSubscriber
        print("✓ TFSubscriber imported successfully")
    except Exception as e:
        print(f"✗ TFSubscriber import failed: {e}")
        return False
    
    return True

def test_basic_functionality():
    print("\nTesting basic functionality...")
    
    try:
        from classes.controllers.MapState import MapState
        from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator
        
        # Test MapState
        map_state = MapState()
        map_state.add_location("test_location", (1.0, 2.0))
        location = map_state.get_location("test_location")
        assert location == (1.0, 2.0), f"Expected (1.0, 2.0), got {location}"
        print("✓ MapState basic functionality works")
        
        # Test object detection storage
        map_state.add_detected_object("chair", (3.0, 4.0), confidence=0.8)
        chairs = map_state.get_objects_by_class("chair")
        assert len(chairs) == 1, f"Expected 1 chair, got {len(chairs)}"
        assert chairs[0]['position'] == (3.0, 4.0), f"Position mismatch"
        assert chairs[0]['confidence'] == 0.8, f"Confidence mismatch"
        print("✓ MapState object detection storage works")
        
        # Test position estimator utility functions
        estimator = ObjectPositionEstimator(None)  # No TF subscriber for basic test
        
        # Test pixel to angle conversion
        center_x, center_y = estimator.get_object_center_pixel({'x1': 100, 'y1': 100, 'x2': 200, 'y2': 200})
        assert center_x == 150 and center_y == 150, f"Center calculation failed"
        print("✓ ObjectPositionEstimator bounding box center calculation works")
        
        # Test angular conversion
        h_angle = estimator.pixel_to_angle(320, 240)  # Center of 640x480 image
        assert abs(h_angle) < 0.01, f"Center pixel should have near-zero angle"
        print("✓ ObjectPositionEstimator pixel-to-angle conversion works")
        
        # Test precision info
        precision_info = estimator.get_angular_precision_info()
        assert 'camera_horizontal_resolution_rad_per_pixel' in precision_info
        print("✓ ObjectPositionEstimator precision info generation works")
        
        return True
    
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False

def main():
    print("Object Position Estimation - Component Test")
    print("=" * 50)
    
    if not test_imports():
        print("\n❌ Import tests failed. Please check dependencies.")
        return 1
    
    if not test_basic_functionality():
        print("\n❌ Basic functionality tests failed.")
        return 1
    
    print("\n✅ All tests passed! Object position estimation components are ready.")
    print("\nNext steps:")
    print("1. Run the main application with ROS2 environment")
    print("2. Ensure camera and LIDAR topics are available")
    print("3. Check TF transforms for camera frames")
    print("4. Use demo_object_position_estimation.py for testing")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
