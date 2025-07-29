from typing import Dict, List, Tuple, Optional
import time
import math


class MapState():
    def __init__(self):
        self.locations = {}
        self.detected_objects = {}  # Store detected objects with their 2D positions and metadata
        
    def add_location(self, name: str, position: Tuple[float, float]):
        """Add a named location to the map."""
        self.locations[name] = position

    def get_location(self, name: str) -> Optional[Tuple[float, float]]:
        """Get a named location from the map."""
        return self.locations.get(name)

    def get_all_locations(self) -> Dict[str, Tuple[float, float]]:
        """Get all named locations."""
        return self.locations
    
    def add_detected_object(self, object_class: str, position: Tuple[float, float], 
                          confidence: float = 1.0, metadata: Dict = None):
        """
        Add a detected object to the map with its estimated 2D world position.
        
        Args:
            object_class: Class name of the detected object
            position: World position (x, y) of the object
            confidence: Detection confidence (0.0 to 1.0)
            metadata: Additional metadata about the detection
        """
        timestamp = time.time()
        
        if object_class not in self.detected_objects:
            self.detected_objects[object_class] = []
        
        object_info = {
            'position': position,
            'confidence': confidence,
            'timestamp': timestamp,
            'metadata': metadata or {}
        }
        
        self.detected_objects[object_class].append(object_info)
        
        # Keep only recent detections (last 100 detections per class)
        if len(self.detected_objects[object_class]) > 100:
            self.detected_objects[object_class] = self.detected_objects[object_class][-100:]
    
    def get_objects_by_class(self, object_class: str) -> List[Dict]:
        """Get all detected objects of a specific class."""
        return self.detected_objects.get(object_class, [])
    
    def get_all_detected_objects(self) -> Dict[str, List[Dict]]:
        """Get all detected objects organized by class."""
        return self.detected_objects
    
    def get_nearest_object(self, object_class: str, reference_position: Tuple[float, float], 
                          max_age_seconds: float = 60.0) -> Optional[Dict]:
        """
        Find the nearest detected object of a specific class to a reference position.
        
        Args:
            object_class: Class of object to search for
            reference_position: Reference position (x, y)
            max_age_seconds: Maximum age of detection to consider
            
        Returns:
            Dictionary with object info or None if not found
        """
        objects = self.get_objects_by_class(object_class)
        current_time = time.time()
        
        nearest_object = None
        min_distance = float('inf')
        
        for obj in objects:
            # Skip old detections
            if current_time - obj['timestamp'] > max_age_seconds:
                continue
                
            # Calculate 2D distance
            obj_pos = obj['position']
            distance = math.sqrt(
                (obj_pos[0] - reference_position[0]) ** 2 +
                (obj_pos[1] - reference_position[1]) ** 2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_object = obj.copy()
                nearest_object['distance'] = distance
        
        return nearest_object
    
    def get_objects_in_radius(self, center_position: Tuple[float, float], 
                            radius: float, object_class: str = None, 
                            max_age_seconds: float = 60.0) -> List[Dict]:
        """
        Get all objects within a certain radius of a position.
        
        Args:
            center_position: Center position (x, y)
            radius: Search radius in meters
            object_class: Specific object class to search for (None for all)
            max_age_seconds: Maximum age of detection to consider
            
        Returns:
            List of objects within radius
        """
        current_time = time.time()
        objects_in_radius = []
        
        classes_to_search = [object_class] if object_class else self.detected_objects.keys()
        
        for cls in classes_to_search:
            for obj in self.get_objects_by_class(cls):
                # Skip old detections
                if current_time - obj['timestamp'] > max_age_seconds:
                    continue
                    
                # Calculate 2D distance
                obj_pos = obj['position']
                distance = math.sqrt(
                    (obj_pos[0] - center_position[0]) ** 2 +
                    (obj_pos[1] - center_position[1]) ** 2
                )
                
                if distance <= radius:
                    obj_copy = obj.copy()
                    obj_copy['class'] = cls
                    obj_copy['distance'] = distance
                    objects_in_radius.append(obj_copy)
        
        # Sort by distance
        objects_in_radius.sort(key=lambda x: x['distance'])
        return objects_in_radius
    
    def remove_old_detections(self, max_age_seconds: float = 300.0):
        """Remove detections older than specified age."""
        current_time = time.time()
        
        for object_class in self.detected_objects:
            self.detected_objects[object_class] = [
                obj for obj in self.detected_objects[object_class]
                if current_time - obj['timestamp'] <= max_age_seconds
            ]
    
    def get_detection_statistics(self) -> Dict[str, Dict]:
        """Get statistics about detected objects."""
        stats = {}
        
        for object_class, objects in self.detected_objects.items():
            if not objects:
                continue
                
            confidences = [obj['confidence'] for obj in objects]
            timestamps = [obj['timestamp'] for obj in objects]
            
            stats[object_class] = {
                'count': len(objects),
                'avg_confidence': sum(confidences) / len(confidences),
                'min_confidence': min(confidences),
                'max_confidence': max(confidences),
                'latest_detection': max(timestamps),
                'oldest_detection': min(timestamps)
            }
        
        return stats
