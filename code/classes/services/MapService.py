import time

class MapService:
    """Service to manage persistent world object mapping and tracking"""

    def __init__(self):
        # Persistent world object map to store all detected objects and their positions
        self.world_object_map = {}

        # Object persistence settings
        self.object_update_distance_threshold = 0.5  # Objects within 50cm are considered the same
        self.object_confidence_threshold = 5  # Object must be seen 5 times to be added to persistent map

        # Callbacks
        self.coordinate_update_callbacks = []

    def add_coordinate_update_callback(self, callback):
        """Add a callback to be notified when coordinates are updated"""
        self.coordinate_update_callbacks.append(callback)

    def remove_coordinate_update_callback(self, callback):
        """Remove a coordinate update callback"""
        if callback in self.coordinate_update_callbacks:
            self.coordinate_update_callbacks.remove(callback)

    def update_world_object_map(self, new_object_coordinates):
        """Update the persistent world object map with new coordinate data"""
        current_time = time.time()

        for object_class, new_objects in new_object_coordinates.items():
            # Initialize object class if not exists
            if object_class not in self.world_object_map:
                self.world_object_map[object_class] = []

            for new_obj in new_objects:
                # Handle objects with or without world coordinates
                world_coords = new_obj.get('world_coords')

                # Try to find existing object nearby
                matched_existing = False

                if world_coords:
                    # Use world coordinates for matching if available
                    new_x, new_y, new_z = world_coords
                    for existing_obj in self.world_object_map[object_class]:
                        if existing_obj.get('world_coords'):
                            ex_x, ex_y, ex_z = existing_obj['world_coords']
                            distance = ((new_x - ex_x)**2 + (new_y - ex_y)**2)**0.5

                            if distance < self.object_update_distance_threshold:
                                # Update position of existing object
                                existing_obj['world_coords'] = (new_x, new_y, new_z)
                                existing_obj['camera_coords'] = new_obj.get('camera_coords')
                                existing_obj['distance_mm'] = new_obj.get('distance_mm')
                                existing_obj['detection'] = new_obj.get('detection')
                                existing_obj['is_selected_target'] = new_obj.get('is_selected_target', False)
                                existing_obj['last_seen'] = current_time
                                existing_obj['confidence'] = min(existing_obj.get('confidence', 0) + 1, 10)
                                matched_existing = True
                                break
                else:
                    # Fallback: try to match by bounding box hash for objects without world coords
                    bbox_hash = new_obj.get('bbox_hash')
                    if bbox_hash:
                        for existing_obj in self.world_object_map[object_class]:
                            if existing_obj.get('bbox_hash') == bbox_hash:
                                # Update existing object (but don't update world_coords if it was None)
                                existing_obj['camera_coords'] = new_obj.get('camera_coords')
                                existing_obj['distance_mm'] = new_obj.get('distance_mm')
                                existing_obj['detection'] = new_obj.get('detection')
                                existing_obj['is_selected_target'] = new_obj.get('is_selected_target', False)
                                existing_obj['last_seen'] = current_time
                                existing_obj['confidence'] = min(existing_obj.get('confidence', 0) + 1, 10)
                                matched_existing = True
                                break

                if not matched_existing:
                    # Add new object to persistent map (even without world coordinates)
                    persistent_obj = {
                        'world_coords': world_coords,  # Can be None
                        'camera_coords': new_obj.get('camera_coords'),
                        'distance_mm': new_obj.get('distance_mm'),
                        'detection': new_obj.get('detection'),
                        'is_selected_target': new_obj.get('is_selected_target', False),
                        'object_id': len(self.world_object_map[object_class]),  # Unique persistent ID
                        'bbox_hash': new_obj.get('bbox_hash'),
                        'first_seen': current_time,
                        'last_seen': current_time,
                        'confidence': 1
                    }
                    self.world_object_map[object_class].append(persistent_obj)

        # Notify all callbacks about the update
        for callback in self.coordinate_update_callbacks:
            callback(self.world_object_map)

    def get_world_object_map(self):
        """Get the current world object map"""
        return self.world_object_map

    def get_objects_by_type(self, object_type):
        """Get all objects of a specific type with their world coordinates"""
        return self.world_object_map.get(object_type, [])

    def clear_persistent_map(self):
        """Clear all persistent map data (objects only)"""
        self.world_object_map = {}

        # Notify callbacks about the clear
        for callback in self.coordinate_update_callbacks:
            callback(self.world_object_map)

    def get_map_statistics(self):
        """Get statistics about the persistent map"""
        object_count = sum(len(objects) for objects in self.world_object_map.values())

        return {
            'total_objects': object_count,
            'object_types': len(self.world_object_map)
        }

    def remove_old_objects(self, max_age_seconds=30):
        """Remove objects that haven't been seen for a while"""
        current_time = time.time()

        for object_class in list(self.world_object_map.keys()):
            objects_to_keep = []
            for obj in self.world_object_map[object_class]:
                age = current_time - obj.get('last_seen', current_time)
                if age < max_age_seconds:
                    objects_to_keep.append(obj)

            self.world_object_map[object_class] = objects_to_keep

            # Remove empty object classes
            if not self.world_object_map[object_class]:
                del self.world_object_map[object_class]