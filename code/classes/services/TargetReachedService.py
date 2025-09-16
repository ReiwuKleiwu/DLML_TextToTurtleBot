import time
import math
from classes.events import EventQueue, EventType, Event
from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource


class TargetReachedService:
    """Service that determines when the robot has reached its target object"""

    def __init__(self, state_machine: StateMachine, reach_distance_threshold=1.5):
        """
        Initialize the target reached service.

        Args:
            state_machine: StateMachine instance to update when target is reached
            reach_distance_threshold: Distance in meters to consider target as reached
        """
        self.state_machine = state_machine
        self.reach_distance_threshold = reach_distance_threshold

        # Robot state
        self.robot_position = None  # (x, y, z) in world coordinates
        self.current_target_object = None
        self.target_reached = False

        # Persistent object data from map service
        self.persistent_objects_map = {}

        # Event system
        self.event_queue = EventQueue()

        # Subscribe to relevant events
        self.event_queue.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)
        self.event_queue.subscribe(EventType.TARGET_SELECTED, self._on_target_selected)
        self.event_queue.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)

    def _on_robot_transform_updated(self, event: Event):
        """Handle robot position updates"""
        self.robot_position = event.data.get('position')
        # Check target proximity whenever robot position updates
        self._check_target_proximity()

    def _on_target_selected(self, event: Event):
        """Handle target selection events from TargetSelector"""
        if event.source == "TargetSelector":
            new_target = event.data.get('target_object_class')
            selection_event = event.data.get('selection_event')

            # Reset target reached status when target changes
            if new_target != self.current_target_object:
                self.current_target_object = new_target
                self.target_reached = False
                print(f"[TargetReachedService] Target {selection_event}: {self.current_target_object}")

    def _on_map_updated(self, event: Event):
        """Handle map service updates with persistent object data"""
        if event.source == "MapService":
            self.persistent_objects_map = event.data.get('persistent_objects_map', {})
            # Check target proximity when map updates
            self._check_target_proximity()

    def _check_target_proximity(self):
        """Check if the robot is within reach distance of the target object"""

        # Need all required data to perform check
        if not all([self.robot_position, self.current_target_object, self.persistent_objects_map]):
            return

        # Skip if target already reached
        if self.target_reached:
            return

        # Get target objects from persistent map
        if self.current_target_object not in self.persistent_objects_map:
            return

        target_objects = self.persistent_objects_map[self.current_target_object]

        # Find the SELECTED target object (not just the closest one)
        selected_target = None
        selected_distance = float('inf')

        for target_obj in target_objects:
            # Only check the object that is marked as the selected target
            if target_obj.get('is_selected_target', False) and target_obj.get('world_coords'):
                distance = self._calculate_distance(self.robot_position, target_obj['world_coords'])
                selected_target = target_obj
                selected_distance = distance
                break  # There should only be one selected target

        # Check if selected target is within reach
        if selected_target and selected_distance <= self.reach_distance_threshold:
            self._target_reached(selected_target, selected_distance)

    def _calculate_distance(self, pos1, pos2):
        """Calculate 3D Euclidean distance between two positions"""
        if not pos1 or not pos2:
            return float('inf')

        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]

        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _target_reached(self, target_object, distance):
        """Handle target reached by popping states and publishing event"""
        self.target_reached = True

        # Pop OBJECT_FOUND and EXPLORE states
        self.state_machine.pop_state(
            TurtleBotState.OBJECT_FOUND,
            TurtleBotStateSource.CAMERA
        )
        self.state_machine.pop_state(
            TurtleBotState.EXPLORE,
            TurtleBotStateSource.USER
        )

        # Publish target reached event for other services to react to
        target_reached_data = {
            'target_object_class': self.current_target_object,
            'target_world_coords': target_object['world_coords'],
            'robot_position': self.robot_position,
            'distance': distance,
            'reach_threshold': self.reach_distance_threshold,
            'timestamp': time.time()
        }

        self.event_queue.publish_event(
            EventType.TARGET_REACHED,
            source="TargetReachedService",
            data=target_reached_data
        )

        print(f"Target reached! {self.current_target_object} at distance {distance:.2f}m (world coordinates)")

        # Reset target
        self.current_target_object = None
        self.target_reached = False

    def reset_target_status(self):
        """Reset target reached status (useful when starting new navigation)"""
        self.target_reached = False
        self.current_target_object = None

    def is_target_reached(self):
        """Check if current target has been reached"""
        return self.target_reached

    def get_current_target(self):
        """Get the current target object class"""
        return self.current_target_object

    def set_reach_distance_threshold(self, distance):
        """Update the reach distance threshold"""
        self.reach_distance_threshold = distance

    def get_status(self):
        """Get current service status"""
        return {
            'target_reached': self.target_reached,
            'current_target': self.current_target_object,
            'robot_position': self.robot_position,
            'reach_threshold': self.reach_distance_threshold,
            'persistent_objects_available': len(self.persistent_objects_map)
        }