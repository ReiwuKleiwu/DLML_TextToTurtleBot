import math
from typing import Dict, List, Optional, Tuple
from code.core.events import EventType
from code.core.interfaces.event_bus import EventBus


class TargetSelector:
    """
    Handles target selection and persistence when multiple objects of the same class are detected.
    Implements strategies to prevent oscillation between targets.
    """
    
    def __init__(
        self,
        event_bus: EventBus,
        persistence_frames: int = 10,
        distance_threshold: float = 50.0,
    ):
        """
        Initialize the target selector.

        Args:
            persistence_frames: Number of frames to persist with a target before allowing switching
            distance_threshold: Maximum pixel distance for considering targets as "the same"
        """
        self.persistence_frames = persistence_frames
        self.distance_threshold = distance_threshold
        self.selected_target = None
        self.selected_target_info = None
        self.frames_since_selection = 0
        self.lost_target_frames = 0
        self.max_lost_frames = 5
        self.current_target_class = None

        # Tracking ID for persistent object identification
        self.current_tracking_id = None
        self._next_tracking_id = 1

        self._bus = event_bus
    
    def select_target(self, target_class: str, detections: List[Dict]) -> Optional[Dict]:
        """
        Select the best target from multiple detections of the same class.

        Args:
            target_class: The class of object to select (e.g., "chair")
            detections: List of detection dictionaries with bounding box coordinates

        Returns:
            Selected target detection dictionary or None
        """
        # Store target class for events
        self.current_target_class = target_class

        if not detections:
            if self.selected_target is not None:
                self.lost_target_frames += 1
                if self.lost_target_frames >= self.max_lost_frames:
                    self._clear_selection()
            return None

        # Reset lost frames counter since we have detections
        self.lost_target_frames = 0

        # If no target is currently selected, select the best one
        if self.selected_target is None:
            best_target = self._select_best_initial_target(detections)
            self._set_selected_target(best_target)
            return best_target

        # Try to find the same target in current detections
        current_target = self._find_matching_target(detections)

        if current_target is not None:
            # Update the selected target info and continue tracking
            self.selected_target_info = current_target
            self.frames_since_selection += 1
            return current_target

        # If we lost the target but haven't exceeded persistence threshold
        if self.frames_since_selection < self.persistence_frames:
            self.frames_since_selection += 1
            # Return the last known position to maintain course
            return self.selected_target_info

        # Persistence threshold exceeded, select new target
        best_target = self._select_best_initial_target(detections)
        self._set_selected_target(best_target)
        return best_target
    
    def _select_best_initial_target(self, detections: List[Dict]) -> Dict:
        """
        Select the best target from multiple detections using selection criteria.
        Current strategy: Choose the largest (closest) object.
        """
        best_target = None
        best_score = -1
        
        for detection in detections:
            # Calculate area as a proxy for distance (larger = closer)
            area = (detection['x2'] - detection['x1']) * (detection['y2'] - detection['y1'])
            
            # TODO: Future criteria could include:
            # - Centrality (objects closer to center of image)
            # - Stability (objects that appear in multiple consecutive frames)
            # - Distance from robot's current orientation
            
            if area > best_score:
                best_score = area
                best_target = detection
        
        return best_target
    
    def _find_matching_target(self, detections: List[Dict]) -> Optional[Dict]:
        """
        Find the detection that matches the currently selected target.
        """
        if self.selected_target_info is None:
            return None
        
        best_match = None
        best_distance = float('inf')
        
        # Calculate center of currently selected target
        selected_center_x = (self.selected_target_info['x1'] + self.selected_target_info['x2']) / 2
        selected_center_y = (self.selected_target_info['y1'] + self.selected_target_info['y2']) / 2
        
        for detection in detections:
            # Calculate center of candidate detection
            candidate_center_x = (detection['x1'] + detection['x2']) / 2
            candidate_center_y = (detection['y1'] + detection['y2']) / 2
            
            # Calculate distance between centers
            distance = math.sqrt(
                (selected_center_x - candidate_center_x) ** 2 + 
                (selected_center_y - candidate_center_y) ** 2
            )
            
            if distance < self.distance_threshold and distance < best_distance:
                best_distance = distance
                best_match = detection
        
        return best_match
    
    def _set_selected_target(self, target: Dict):
        """Set the selected target and assign new tracking ID."""
        self.selected_target = target
        self.selected_target_info = target
        self.frames_since_selection = 0
        self.lost_target_frames = 0
        # Assign new tracking ID when selecting a new target
        self.current_tracking_id = self._next_tracking_id
        self._next_tracking_id += 1

        # Broadcast target selection event
        self._broadcast_target_selected()

    def _clear_selection(self):
        """Clear the currently selected target."""
        self.selected_target = None
        self.selected_target_info = None
        self.frames_since_selection = 0
        self.lost_target_frames = 0
        self.current_tracking_id = None

        # Broadcast target cleared event
        self._broadcast_target_cleared()

    def reset(self):
        """Reset the target selector to initial state."""
        self._clear_selection()

    def get_selected_target_info(self) -> Optional[Dict]:
        """Get information about the currently selected target."""
        return self.selected_target_info

    def get_current_tracking_id(self) -> Optional[int]:
        """Get the tracking ID of the currently selected target."""
        return self.current_tracking_id

    def _broadcast_target_selected(self):
        """Broadcast target selection event"""
        if self.current_target_class and self.selected_target_info:
            self._bus.publish_event(
                EventType.TARGET_SELECTED,
                source="TargetSelector",
                data={
                    'target_object_class': self.current_target_class,
                    'target_info': self.selected_target_info,
                    'tracking_id': self.current_tracking_id,
                    'selection_event': 'selected'
                }
            )

    def _broadcast_target_cleared(self):
        """Broadcast target cleared event"""
        self._bus.publish_event(
            EventType.TARGET_SELECTED,
            source="TargetSelector",
            data={
                'target_object_class': None,
                'target_info': None,
                'tracking_id': None,
                'selection_event': 'cleared'
            }
        )
