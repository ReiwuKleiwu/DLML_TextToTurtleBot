from classes.visualization.InteractiveMapVisualizer import InteractiveMapVisualizer
from classes.events import EventQueue, EventType, Event


class VisualizationService:
    """Event-driven visualization service that reacts to map and sensor events"""

    def __init__(self, window_size=(1000, 800), world_bounds=(-5.0, -5.0, 5.0, 5.0)):
        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        # Data storage for visualization
        self.persistent_objects_map = {}
        self.lidar_scan_data = {}
        self.robot_position = None
        self.robot_orientation = None
        self.current_target_object = None
        self.robot_state = "UNKNOWN"

        # Initialize interactive map visualizer
        self.map_visualizer = InteractiveMapVisualizer(
            window_size=window_size,
            initial_world_bounds=world_bounds
        )

        # Start the visualization in a separate thread
        self.map_visualizer.start()

        # Subscribe to all relevant events
        self.event_queue.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)
        self.event_queue.subscribe(EventType.LIDAR_SCAN_PROCESSED, self._on_lidar_updated)
        self.event_queue.subscribe(EventType.STATE_CHANGED, self._on_state_changed)
        self.event_queue.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)

    def _on_map_updated(self, event: Event):
        """Handle map service updates with persistent object data"""
        if event.source == "MapService":
            self.persistent_objects_map = event.data.get('persistent_objects_map', {})
            self._update_visualization()

    def _on_lidar_updated(self, event: Event):
        """Handle LIDAR scan updates"""
        self.lidar_scan_data = {
            'scan_points': event.data.get('scan_points', []),
            'obstacle_points': event.data.get('obstacle_points', []),
            'timestamp': event.data.get('timestamp')
        }
        self._update_visualization()

    def _on_state_changed(self, event: Event):
        """Handle state machine changes"""
        if event.source == "StateMachine":
            self.robot_state = event.data.get('current_state', "UNKNOWN")
            # Extract target object from state data if available
            state_data = event.data.get('state_data', {})
            if state_data and 'target_object' in state_data:
                self.current_target_object = state_data['target_object']
            self._update_visualization()

    def _update_visualization(self):
        """Update the map visualization with current data"""
        if not self.map_visualizer.is_running():
            return

        # Convert persistent objects to visualization format
        world_object_map = {}
        total_objects = 0

        for object_class, objects_list in self.persistent_objects_map.items():
            world_object_map[object_class] = []
            for obj in objects_list:
                world_object_map[object_class].append({
                    'world_coords': obj.get('world_coords'),
                    'detection_count': obj.get('detection_count', 0),
                    'last_seen': obj.get('last_seen', 0),
                    'first_seen': obj.get('first_seen', 0),
                    'is_selected_target': obj.get('is_selected_target', False)  # Use the actual selected target flag
                })
                total_objects += 1

        # Prepare statistics
        object_counts = {}
        for obj_class, obj_list in world_object_map.items():
            object_counts[obj_class] = len(obj_list)

        additional_info = {
            'object_count': object_counts,
            'robot_state': self.robot_state,
            'target_object': self.current_target_object,
            'total_objects': total_objects,
            'lidar_points': len(self.lidar_scan_data.get('scan_points', []))
        }

        # Update the visualization
        self.map_visualizer.update_visualization_data(
            robot_position=self.robot_position,
            robot_orientation=self.robot_orientation,
            world_object_map=world_object_map,
            target_object_class=self.current_target_object,
            lidar_scan_data=self.lidar_scan_data,
            additional_info=additional_info
        )

    def _on_robot_transform_updated(self, event: Event):
        """Handle robot transform updates"""
        self.robot_position = event.data.get('position')
        self.robot_orientation = event.data.get('orientation')
        self._update_visualization()

    def clear_map(self):
        """Clear the persistent map (callback for visualizer clear button)"""
        # Publish event to clear the map
        self.event_queue.publish_event(
            EventType.SENSOR_DATA_UPDATED,
            source="VisualizationService",
            data={'clear_map_request': True}
        )

    def is_running(self):
        """Check if the map visualizer is running"""
        return self.map_visualizer.is_running()

    def stop(self):
        """Stop the visualization"""
        if hasattr(self, 'map_visualizer'):
            self.map_visualizer.stop()