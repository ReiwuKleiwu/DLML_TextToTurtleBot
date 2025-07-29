# Object Position Estimation with Camera-LIDAR Fusion (2D)

This document describes the 2D object position estimation system that combines camera object detection with LIDAR distance measurements to determine the world coordinates (x, y) of detected objects.

## Overview

The system uses the following approach:

1. **Object Detection**: YOLO model detects objects in camera images and provides bounding boxes
2. **Angle Calculation**: Convert pixel coordinates to precise horizontal angular coordinates relative to camera
3. **Distance Measurement**: Use LIDAR to measure distance at the calculated angle
4. **Position Estimation**: Combine angle, distance, and robot pose to calculate 2D world coordinates
5. **Map Storage**: Save estimated positions to MapState for navigation and planning

## Key Components

### ObjectPositionEstimator

Located in `classes/perception/ObjectPositionEstimator.py`

**Key Features:**

- Precise pixel-to-angle conversion using camera horizontal field of view
- LIDAR distance lookup at specific angles
- Support for camera frame transforms for higher accuracy
- Quaternion to yaw conversion for orientation handling
- Confidence calculation based on detection quality
- **2D Focus**: Only estimates X,Y coordinates for ground-plane navigation

**Key Methods:**

- `pixel_to_angle()`: Convert bounding box center to horizontal angular coordinate
- `get_lidar_distance_at_angle()`: Get LIDAR distance measurement at specific angle
- `estimate_object_position_with_camera_tf()`: Main 2D position estimation using camera TF
- `get_angular_precision_info()`: System precision metrics

### Enhanced MapState

Located in `classes/controllers/MapState.py`

**New Features:**

- Store detected objects with 2D positions, confidence, and metadata
- Query objects by class, proximity, or age
- Automatic cleanup of old detections
- Detection statistics and analytics

**Key Methods:**

- `add_detected_object()`: Add object with 2D world position
- `get_nearest_object()`: Find closest object of specific class
- `get_objects_in_radius()`: Get all objects within distance
- `get_detection_statistics()`: Analytics about detections

### Enhanced CameraHandler

The CameraHandler now integrates 2D position estimation:

- Estimates 2D positions for all detected objects
- Saves positions to MapState automatically
- Calculates confidence scores based on detection quality
- Provides detailed logging of position estimates

## Angular Precision

The system achieves high angular precision through:

### Camera Resolution

- **Horizontal FOV**: 69.4° (typical for OAK-D Pro)
- **Resolution**: 640x480 pixels (configurable)
- **Angular Resolution**: ~0.108° per pixel horizontally

### LIDAR Integration

- Uses LIDAR angular increment for precise distance measurement
- Matches camera angles to LIDAR beam directions
- Filters invalid LIDAR readings (inf, nan, zero)

### Transform Accuracy

- Uses TF2 transforms for precise camera positioning
- Supports camera-specific frames (e.g., `oakd_rgb_camera_optical_frame`)
- Fallback to robot base_link if camera frame unavailable

## Usage Example

```python
from classes.nodes.TextToTurtlebotNode import TextToTurtlebotNode

# Create node with 2D position estimation
node = TextToTurtlebotNode(use_turtlebot_sim=True)

# Start exploring for chairs
node.find_target("chair")

# Get detected objects
chairs = node.get_detected_objects("chair")
for chair in chairs:
    pos = chair['position']
    confidence = chair['confidence']
    print(f"Chair at ({pos[0]:.2f}, {pos[1]:.2f}) confidence: {confidence:.2f}")

# Find nearest chair to robot
nearest_chair = node.get_nearest_object("chair")
if nearest_chair:
    distance = nearest_chair['distance']
    print(f"Nearest chair is {distance:.2f}m away")

# Get all objects within 3 meters
nearby = node.get_objects_in_radius(3.0)
print(f"Found {len(nearby)} objects within 3 meters")
```

## Configuration

### Camera Parameters

Update camera parameters in ObjectPositionEstimator:

```python
estimator.update_camera_params(
    width=640,
    height=480,
    fov_h=math.radians(69.4)  # Horizontal FOV
)
```

### Frame Names

Set appropriate frame names for your robot:

```python
position_estimator = ObjectPositionEstimator(
    tf_subscriber=tf_subscriber,
    camera_frame="oakd_rgb_camera_optical_frame"  # Adjust as needed
)
```

## Accuracy Considerations

### Factors Affecting Accuracy

1. **Camera Calibration**: Proper horizontal field of view value is critical
2. **LIDAR Quality**: Distance measurement accuracy affects position
3. **TF Synchronization**: Accurate robot pose and camera transforms
4. **Object Size**: Larger objects provide more stable position estimates
5. **Distance**: Accuracy decreases with distance due to angular resolution

### Best Practices

1. Use camera-specific TF frames when available
2. Filter detections by confidence score
3. Use multiple measurements for stable objects
4. Clean up old detections regularly
5. Validate positions against known landmarks

## Troubleshooting

### Common Issues

1. **No positions estimated**: Check LIDAR data availability and TF transforms
2. **Inaccurate positions**: Verify camera horizontal FOV parameter and frame names
3. **Missing detections**: Ensure objects are within both camera and LIDAR range
4. **TF errors**: Check that all required frames are being published

### Debug Information

Enable debug output to see:

- Angular precision metrics
- Transform availability
- Position estimation success/failure
- Confidence calculations

## Integration with Navigation

The estimated 2D object positions can be used for:

- **Goal Setting**: Navigate to specific object locations
- **Path Planning**: Avoid detected obstacles
- **Mapping**: Build semantic maps with object locations
- **Task Planning**: Plan actions based on object positions

Example navigation to nearest chair:

```python
nearest_chair = node.get_nearest_object("chair")
if nearest_chair:
    goal_position = nearest_chair['position']  # (x, y) tuple
    # Use navigation system to go to goal_position
```
