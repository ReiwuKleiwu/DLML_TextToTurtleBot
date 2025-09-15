from classes.visualization.InteractiveMapVisualizer import InteractiveMapVisualizer

class VisualizationService:
    """Service to manage map visualization and coordinate updates between components"""

    def __init__(self, map_service, tf_subscriber, state_machine, window_size=(1000, 800), world_bounds=(-5.0, -5.0, 5.0, 5.0)):
        self.map_service = map_service
        self.tf_subscriber = tf_subscriber
        self.state_machine = state_machine

        # Store latest LIDAR scan data (real-time only)
        self.latest_lidar_data = {}

        # Initialize interactive map visualizer
        self.map_visualizer = InteractiveMapVisualizer(
            window_size=window_size,
            initial_world_bounds=world_bounds
        )

        # Start the visualization in a separate thread
        self.map_visualizer.start()

        # Connect the clear map callback
        self.map_visualizer.set_clear_map_callback(self.map_service.clear_persistent_map)

    def update_lidar_data(self, lidar_scan_data):
        """Update LIDAR scan data for real-time visualization"""
        # Use real-time LIDAR data only (no accumulation)
        self.latest_lidar_data = lidar_scan_data

    def get_map_statistics(self):
        """Get statistics about the persistent map"""
        map_stats = self.map_service.get_map_statistics()
        current_lidar_points = len(self.latest_lidar_data.get('scan_points', []))

        return {
            'total_objects': map_stats['total_objects'],
            'object_types': map_stats['object_types'],
            'lidar_points': current_lidar_points
        }

    def update_map_visualization(self, clock):
        """Update the map visualization with current robot state and object positions"""
        # Get TurtleBot position and orientation
        turtlebot_position = self.tf_subscriber.get_position()
        turtlebot_orientation = self.tf_subscriber.get_orientation()

        # Get current target object
        current_state = self.state_machine.get_current_state()
        target_object_class = None
        robot_state = "UNKNOWN"

        if current_state:
            robot_state = current_state.value.name
            if hasattr(current_state, 'data') and current_state.data:
                target_object_class = current_state.data.get("target_object", None)

        # Prepare additional info for display
        world_object_map = self.map_service.get_world_object_map()
        object_counts = {}
        for obj_class, obj_list in world_object_map.items():
            object_counts[obj_class] = len(obj_list)

        map_stats = self.get_map_statistics()

        additional_info = {
            'object_count': object_counts,
            'robot_state': robot_state,
            'target_object': target_object_class,
            'timestamp': clock.now().to_msg().sec,
            'map_stats': map_stats
        }

        # Update the map data (thread-safe)
        if self.map_visualizer.is_running():
            self.map_visualizer.update_data(
                turtlebot_position=turtlebot_position,
                turtlebot_orientation=turtlebot_orientation,
                world_object_map=world_object_map,
                target_object_class=target_object_class,
                additional_info=additional_info,
                lidar_data=self.latest_lidar_data
            )

    def is_running(self):
        """Check if the map visualizer is running"""
        return self.map_visualizer.is_running()

    def stop(self):
        """Stop the visualization"""
        if hasattr(self, 'map_visualizer'):
            self.map_visualizer.stop()