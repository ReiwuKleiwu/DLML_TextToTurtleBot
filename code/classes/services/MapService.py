import time
import math
from classes.events import EventQueue, EventType, Event


class TemporaryTrackedObject:
    """Represents an object being tracked for consecutive frames before persistence"""

    def __init__(self, object_class, world_coords, detection_info):
        self.object_class = object_class
        self.world_coords = world_coords  # (x, y, z)
        self.detection_info = detection_info
        self.consecutive_frames = 1
        self.first_seen = time.time()
        self.last_seen = time.time()

    def update(self, world_coords, detection_info):
        """Update with new detection data"""
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.consecutive_frames += 1
        self.last_seen = time.time()

    def distance_to(self, world_coords):
        """Calculate 3D distance to given coordinates"""
        if not self.world_coords or not world_coords:
            return float('inf')

        dx = self.world_coords[0] - world_coords[0]
        dy = self.world_coords[1] - world_coords[1]
        dz = self.world_coords[2] - world_coords[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)


class PersistentObject:
    """Represents a confirmed persistent object in the world map"""

    def __init__(self, object_class, world_coords, detection_info):
        self.object_class = object_class
        self.world_coords = world_coords  # (x, y, z)
        self.detection_info = detection_info
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.total_detections = 1

    def update_position(self, world_coords, detection_info):
        """Update with new position and detection data"""
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.last_seen = time.time()
        self.total_detections += 1

    def distance_to(self, world_coords):
        """Calculate 3D distance to given coordinates"""
        if not self.world_coords or not world_coords:
            return float('inf')

        dx = self.world_coords[0] - world_coords[0]
        dy = self.world_coords[1] - world_coords[1]
        dz = self.world_coords[2] - world_coords[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)


class MapService:
    """Advanced event-driven map service with temporary tracking and persistent object storage"""

    def __init__(self):
        # Tracking configuration
        self.consecutive_frames_threshold = 30  # Frames needed before persistence
        self.temporary_match_distance = 0.5     # Distance for frame-to-frame matching (meters)
        self.persistent_match_distance = 1.5    # Distance for re-identification (meters)
        self.temporary_object_timeout = 5.0     # Timeout for temporary objects (seconds)

        # Core data storage
        self.temporary_objects = []   # List of TemporaryTrackedObject instances
        self.persistent_objects = []  # List of PersistentObject instances

        # Ignored object classes
        self.ignored_classes = {'wall', 'ceiling', 'floor'}

        # Robot motion state
        self.robot_is_turning = False
        self.last_robot_orientation = None
        self.orientation_change_threshold = 0.1  # Radians per second to consider "turning"

        # Event system
        self.event_queue = EventQueue()

        # Subscribe to depth camera world coordinate events
        self.event_queue.subscribe(EventType.WORLD_COORDINATES_CALCULATED, self._on_world_coordinates_received)

        # Subscribe to robot transform events to detect turning
        self.event_queue.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)

    def _on_world_coordinates_received(self, event: Event):
        """Handle world coordinates from depth camera handler with advanced tracking"""

        # Skip map updates if robot is turning (only track when moving forward/backward)
        if self.robot_is_turning:
            return

        world_coordinates = event.data.get('world_coordinates', {})

        # Mark all temporary objects as not seen this frame
        for temp_obj in self.temporary_objects:
            temp_obj.frames_not_seen = getattr(temp_obj, 'frames_not_seen', 0) + 1

        # Process each detected object
        for object_class, detected_objects in world_coordinates.items():
            for obj_info in detected_objects:
                world_coords = obj_info.get('world_coords')

                # Only process objects with valid world coordinates
                if world_coords and not any(coord is None for coord in world_coords):
                    self._process_detected_object(object_class, world_coords, obj_info)

        # Clean up old temporary objects and promote qualified ones
        self._cleanup_and_promote_objects()

        # Publish updated map event for camera handler and visualization
        self._publish_map_update()

    def _on_robot_transform_updated(self, event: Event):
        """Handle robot transform updates to detect turning"""
        position = event.data.get('position')
        orientation = event.data.get('orientation')

        if position is not None and orientation is not None:
            self.update_robot_motion_state(position, orientation)

    def _process_detected_object(self, object_class, world_coords, detection_info):
        """Process a detected object for tracking and potential persistence"""

        # Skip ignored object classes
        if object_class.lower() in self.ignored_classes:
            return

        # Try to match with existing temporary tracked objects
        matching_temp_obj = self._find_matching_temporary_object(object_class, world_coords)

        if matching_temp_obj:
            # Update existing temporary object
            matching_temp_obj.update(world_coords, detection_info)
            matching_temp_obj.frames_not_seen = 0  # Reset not seen counter
        else:
            # Create new temporary tracked object
            new_temp_obj = TemporaryTrackedObject(object_class, world_coords, detection_info)
            new_temp_obj.frames_not_seen = 0
            self.temporary_objects.append(new_temp_obj)

    def _find_matching_temporary_object(self, object_class, world_coords):
        """Find matching temporary object by class and proximity"""
        best_match = None
        best_distance = float('inf')

        for temp_obj in self.temporary_objects:
            if temp_obj.object_class == object_class:
                distance = temp_obj.distance_to(world_coords)
                if distance <= self.temporary_match_distance and distance < best_distance:
                    best_distance = distance
                    best_match = temp_obj

        return best_match

    def _cleanup_and_promote_objects(self):
        """Clean up old temporary objects and promote qualified ones to persistent storage"""
        current_time = time.time()
        objects_to_promote = []
        objects_to_keep = []

        # Check temporary objects for promotion or cleanup
        for temp_obj in self.temporary_objects:
            # Remove objects that haven't been seen for too long
            if (current_time - temp_obj.last_seen) > self.temporary_object_timeout:
                continue  # Don't keep this object

            # Promote objects that have been tracked long enough
            if temp_obj.consecutive_frames >= self.consecutive_frames_threshold:
                objects_to_promote.append(temp_obj)
            else:
                objects_to_keep.append(temp_obj)

        # Update temporary objects list
        self.temporary_objects = objects_to_keep

        # Promote qualified objects to persistent storage
        for temp_obj in objects_to_promote:
            self._promote_to_persistent(temp_obj)

    def _promote_to_persistent(self, temp_obj):
        """Promote a temporary object to persistent storage with re-identification"""

        # Try to find matching persistent object (re-identification)
        matching_persistent_obj = self._find_matching_persistent_object(
            temp_obj.object_class, temp_obj.world_coords)

        if matching_persistent_obj:
            # Update existing persistent object position
            matching_persistent_obj.update_position(temp_obj.world_coords, temp_obj.detection_info)
        else:
            # Create new persistent object
            new_persistent_obj = PersistentObject(
                temp_obj.object_class, temp_obj.world_coords, temp_obj.detection_info)
            self.persistent_objects.append(new_persistent_obj)

    def _find_matching_persistent_object(self, object_class, world_coords):
        """Find matching persistent object by class and proximity for re-identification"""
        best_match = None
        best_distance = float('inf')

        for persistent_obj in self.persistent_objects:
            if persistent_obj.object_class == object_class:
                distance = persistent_obj.distance_to(world_coords)
                if distance <= self.persistent_match_distance and distance < best_distance:
                    best_distance = distance
                    best_match = persistent_obj

        return best_match

    def update_robot_motion_state(self, position, orientation):
        """Update robot motion state to detect when robot is turning"""
        import time
        import math
        current_time = time.time()

        # Extract yaw angle from orientation (assuming orientation is a tuple/list)
        if isinstance(orientation, (tuple, list)):
            if len(orientation) == 4:
                # Quaternion (x, y, z, w) - extract yaw
                x, y, z, w = orientation
                yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            elif len(orientation) == 3:
                # Euler angles (roll, pitch, yaw) - use yaw
                yaw = orientation[2]
            else:
                # Single angle
                yaw = float(orientation[0]) if len(orientation) > 0 else 0.0
        else:
            # Single float value
            yaw = float(orientation)

        if self.last_robot_orientation is not None:
            # Calculate orientation change rate
            orientation_diff = abs(yaw - self.last_robot_orientation)
            # Handle wraparound (e.g., from 359° to 1°)
            if orientation_diff > 3.14159:  # π radians (180°)
                orientation_diff = 2 * 3.14159 - orientation_diff

            # Estimate time between updates (assume ~10Hz updates)
            time_diff = 0.1  # 100ms between updates

            orientation_change_rate = orientation_diff / time_diff

            # Update turning state
            if orientation_change_rate > self.orientation_change_threshold:
                self.robot_is_turning = True
            else:
                self.robot_is_turning = False

        self.last_robot_orientation = yaw

    def _publish_map_update(self):
        """Publish map update event with persistent object locations"""
        # Create map data for event
        map_data = {}

        for obj in self.persistent_objects:
            if obj.object_class not in map_data:
                map_data[obj.object_class] = []

            # Extract is_selected_target from detection info
            is_selected_target = False
            if obj.detection_info and 'is_selected_target' in obj.detection_info:
                is_selected_target = obj.detection_info['is_selected_target']

            map_data[obj.object_class].append({
                'world_coords': obj.world_coords,
                'first_seen': obj.first_seen,
                'last_seen': obj.last_seen,
                'total_detections': obj.total_detections,
                'detection_info': obj.detection_info,
                'is_selected_target': is_selected_target
            })

        # Publish the event
        self.event_queue.publish_event(
            EventType.SENSOR_DATA_UPDATED,
            source="MapService",
            data={
                'persistent_objects_map': map_data,
                'total_objects': len(self.persistent_objects),
                'temporary_objects': len(self.temporary_objects),
                'object_classes': list(set(obj.object_class for obj in self.persistent_objects))
            }
        )

    def get_objects_by_class(self, object_class):
        """Get all persistent objects of a specific class"""
        return [obj for obj in self.persistent_objects if obj.object_class == object_class]

    def get_all_objects(self):
        """Get all persistent objects"""
        return self.persistent_objects.copy()

    def get_temporary_objects(self):
        """Get all temporary tracked objects"""
        return self.temporary_objects.copy()

    def clear_map(self):
        """Clear all persistent and temporary objects"""
        self.persistent_objects.clear()
        self.temporary_objects.clear()
        self._publish_map_update()

    def remove_old_persistent_objects(self, max_age_seconds=300):
        """Remove persistent objects that haven't been seen for a while"""
        current_time = time.time()
        initial_count = len(self.persistent_objects)

        self.persistent_objects = [
            obj for obj in self.persistent_objects
            if (current_time - obj.last_seen) < max_age_seconds
        ]

        # Publish update if objects were removed
        if len(self.persistent_objects) != initial_count:
            self._publish_map_update()

    def get_statistics(self):
        """Get comprehensive map statistics"""
        persistent_counts = {}
        temporary_counts = {}

        for obj in self.persistent_objects:
            persistent_counts[obj.object_class] = persistent_counts.get(obj.object_class, 0) + 1

        for obj in self.temporary_objects:
            temporary_counts[obj.object_class] = temporary_counts.get(obj.object_class, 0) + 1

        return {
            'persistent_objects': len(self.persistent_objects),
            'temporary_objects': len(self.temporary_objects),
            'persistent_counts': persistent_counts,
            'temporary_counts': temporary_counts,
            'object_classes': list(set(list(persistent_counts.keys()) + list(temporary_counts.keys()))),
            'tracking_config': {
                'consecutive_frames_threshold': self.consecutive_frames_threshold,
                'temporary_match_distance': self.temporary_match_distance,
                'persistent_match_distance': self.persistent_match_distance,
                'temporary_object_timeout': self.temporary_object_timeout
            }
        }