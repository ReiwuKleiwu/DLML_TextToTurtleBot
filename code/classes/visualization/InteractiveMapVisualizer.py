import pygame
import math
import threading
import time


class InteractiveMapVisualizer:
    """Clean, event-driven interactive map visualizer"""

    def __init__(self, window_size=(1000, 800), initial_world_bounds=(-5.0, -5.0, 5.0, 5.0)):
        pygame.init()

        self.window_size = window_size
        self.screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("TurtleBot Interactive World Map")

        # Map view parameters
        self.view_center_x = 0.0
        self.view_center_y = 0.0
        self.scale = 50.0  # Pixels per meter
        self.min_scale = 10.0
        self.max_scale = 200.0

        # Interaction state
        self.dragging = False
        self.last_mouse_pos = (0, 0)

        # Colors
        self.colors = {
            'background': (50, 50, 50),
            'grid': (80, 80, 80),
            'grid_major': (120, 120, 120),
            'turtlebot': (0, 100, 255),      # Blue
            'target_object': (255, 0, 0),    # Red
            'other_object': (0, 255, 0),     # Green
            'text': (255, 255, 255),         # White
            'trajectory': (255, 128, 128),   # Light red
            'ui_panel': (30, 30, 30),        # Dark gray
            'ui_text': (200, 200, 200),      # Light gray
            'lidar_scan': (100, 150, 255),   # Light blue
            'lidar_obstacle': (255, 150, 100) # Orange
        }

        # Fonts
        self.font_small = pygame.font.Font(None, 20)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_large = pygame.font.Font(None, 32)

        # Data storage
        self.robot_position = None
        self.robot_orientation = None
        self.world_object_map = {}
        self.target_object_class = None
        self.lidar_scan_data = {}
        self.additional_info = {}
        self.trajectory_points = []
        self.max_trajectory_points = 500

        # Threading
        self.running = False
        self.visualization_thread = None
        self.data_lock = threading.Lock()

        # Callbacks
        self.clear_map_callback = None

    def start(self):
        """Start the visualization in a separate thread"""
        self.running = True
        self.visualization_thread = threading.Thread(target=self._run_visualization_loop, daemon=True)
        self.visualization_thread.start()

    def stop(self):
        """Stop the visualization"""
        self.running = False
        if self.visualization_thread:
            self.visualization_thread.join(timeout=1.0)
        pygame.quit()

    def is_running(self):
        """Check if the visualizer is running"""
        return self.running

    def set_clear_map_callback(self, callback):
        """Set callback for clear map button"""
        self.clear_map_callback = callback

    def update_visualization_data(self, robot_position=None, robot_orientation=None,
                                world_object_map=None, target_object_class=None,
                                lidar_scan_data=None, additional_info=None):
        """Update visualization data (thread-safe)"""
        with self.data_lock:
            if robot_position is not None:
                self.robot_position = robot_position
                # Add to trajectory
                if robot_position:
                    self.trajectory_points.append(robot_position)
                    if len(self.trajectory_points) > self.max_trajectory_points:
                        self.trajectory_points.pop(0)

            if robot_orientation is not None:
                self.robot_orientation = robot_orientation

            if world_object_map is not None:
                self.world_object_map = world_object_map

            if target_object_class is not None:
                self.target_object_class = target_object_class

            if lidar_scan_data is not None:
                self.lidar_scan_data = lidar_scan_data

            if additional_info is not None:
                self.additional_info = additional_info

    def _run_visualization_loop(self):
        """Main visualization loop running in separate thread"""
        clock = pygame.time.Clock()

        while self.running:
            try:
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_c and event.mod & pygame.KMOD_CTRL:
                            if self.clear_map_callback:
                                self.clear_map_callback()
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click
                            self.dragging = True
                            self.last_mouse_pos = event.pos
                        elif event.button == 4:  # Scroll up
                            self._zoom_in(event.pos)
                        elif event.button == 5:  # Scroll down
                            self._zoom_out(event.pos)
                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button == 1:
                            self.dragging = False
                    elif event.type == pygame.MOUSEMOTION:
                        if self.dragging:
                            self._pan_view(event.pos)

                # Draw everything
                self._draw_frame()

                # Maintain 60 FPS
                clock.tick(60)

            except Exception as e:
                print(f"[Visualization Error]: {e}")

    def _draw_frame(self):
        """Draw a complete frame"""
        # Clear screen
        self.screen.fill(self.colors['background'])

        # Draw grid
        self._draw_grid()

        with self.data_lock:
            # Draw trajectory
            self._draw_trajectory()

            # Draw LIDAR data
            self._draw_lidar_data()

            # Draw objects
            self._draw_objects()

            # Draw robot
            self._draw_robot()

            # Draw UI
            self._draw_ui()

        # Update display
        pygame.display.flip()

    def _draw_grid(self):
        """Draw coordinate grid"""
        # Calculate grid spacing based on zoom level
        if self.scale > 100:
            grid_spacing = 0.5  # 50cm grid
        elif self.scale > 50:
            grid_spacing = 1.0  # 1m grid
        elif self.scale > 25:
            grid_spacing = 2.0  # 2m grid
        else:
            grid_spacing = 5.0  # 5m grid

        # Calculate visible world bounds
        half_width_world = (self.window_size[0] / 2) / self.scale
        half_height_world = (self.window_size[1] / 2) / self.scale

        min_x = self.view_center_x - half_width_world
        max_x = self.view_center_x + half_width_world
        min_y = self.view_center_y - half_height_world
        max_y = self.view_center_y + half_height_world

        # Draw vertical lines
        start_x = math.floor(min_x / grid_spacing) * grid_spacing
        x = start_x
        while x <= max_x:
            screen_x = int((x - self.view_center_x) * self.scale + self.window_size[0] / 2)
            if 0 <= screen_x <= self.window_size[0]:
                color = self.colors['grid_major'] if abs(x) < 0.01 else self.colors['grid']
                pygame.draw.line(self.screen, color, (screen_x, 0), (screen_x, self.window_size[1]))
            x += grid_spacing

        # Draw horizontal lines
        start_y = math.floor(min_y / grid_spacing) * grid_spacing
        y = start_y
        while y <= max_y:
            screen_y = int((-y - (-self.view_center_y)) * self.scale + self.window_size[1] / 2)
            if 0 <= screen_y <= self.window_size[1]:
                color = self.colors['grid_major'] if abs(y) < 0.01 else self.colors['grid']
                pygame.draw.line(self.screen, color, (0, screen_y), (self.window_size[0], screen_y))
            y += grid_spacing

    def _draw_trajectory(self):
        """Draw robot trajectory"""
        if len(self.trajectory_points) < 2:
            return

        points = []
        for world_pos in self.trajectory_points:
            if world_pos:
                screen_pos = self._world_to_screen(world_pos[0], world_pos[1])
                if self._is_visible(screen_pos):
                    points.append(screen_pos)

        if len(points) >= 2:
            pygame.draw.lines(self.screen, self.colors['trajectory'], False, points, 2)

    def _draw_lidar_data(self):
        """Draw LIDAR scan data"""
        scan_points = self.lidar_scan_data.get('scan_points', [])
        obstacle_points = self.lidar_scan_data.get('obstacle_points', [])

        # Draw scan points
        for point in scan_points:
            if len(point) >= 2:
                screen_pos = self._world_to_screen(point[0], point[1])
                if self._is_visible(screen_pos):
                    pygame.draw.circle(self.screen, self.colors['lidar_scan'], screen_pos, 1)

        # Draw obstacle points (larger)
        for point in obstacle_points:
            if len(point) >= 2:
                screen_pos = self._world_to_screen(point[0], point[1])
                if self._is_visible(screen_pos):
                    pygame.draw.circle(self.screen, self.colors['lidar_obstacle'], screen_pos, 2)

    def _draw_objects(self):
        """Draw detected objects"""
        for object_class, objects in self.world_object_map.items():
            for obj_info in objects:
                world_coords = obj_info.get('world_coords')
                if not world_coords or len(world_coords) < 2:
                    continue

                screen_pos = self._world_to_screen(world_coords[0], world_coords[1])
                if not self._is_visible(screen_pos):
                    continue

                # Determine color and size based on target status
                is_target = obj_info.get('is_selected_target', False)

                if is_target:
                    color = self.colors['target_object']  # RED
                    radius = max(6, int(self.scale * 0.15))
                    border = True
                else:
                    color = self.colors['other_object']  # GREEN
                    radius = max(4, int(self.scale * 0.1))
                    border = False

                # Draw object marker
                pygame.draw.circle(self.screen, color, screen_pos, radius)
                if border:
                    pygame.draw.circle(self.screen, self.colors['text'], screen_pos, radius + 1, 2)

                # Object label (when zoomed in)
                if self.scale > 35:
                    detection_count = obj_info.get('detection_count', 0)
                    label = f"{object_class} [{detection_count}x]"
                    text = self.font_small.render(label, True, self.colors['text'])
                    self.screen.blit(text, (screen_pos[0] + radius + 2, screen_pos[1] - 10))

    def _draw_robot(self):
        """Draw robot position and orientation"""
        if not self.robot_position:
            return

        screen_pos = self._world_to_screen(self.robot_position[0], self.robot_position[1])
        if not self._is_visible(screen_pos):
            return

        # Draw robot as circle
        radius = max(8, int(self.scale * 0.2))
        pygame.draw.circle(self.screen, self.colors['turtlebot'], screen_pos, radius)
        pygame.draw.circle(self.screen, self.colors['text'], screen_pos, radius + 1, 2)

        # Draw orientation arrow
        if self.robot_orientation:
            yaw = self.robot_orientation[2] if len(self.robot_orientation) > 2 else 0
            arrow_length = radius + 10
            end_x = screen_pos[0] + arrow_length * math.cos(yaw)
            end_y = screen_pos[1] - arrow_length * math.sin(yaw)  # Negative because screen Y is flipped
            pygame.draw.line(self.screen, self.colors['text'], screen_pos, (int(end_x), int(end_y)), 3)

    def _draw_ui(self):
        """Draw UI information panel"""
        panel_width = 250
        panel_height = 200
        panel_x = self.window_size[0] - panel_width - 10
        panel_y = 10

        # Draw panel background
        panel_rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
        pygame.draw.rect(self.screen, self.colors['ui_panel'], panel_rect)
        pygame.draw.rect(self.screen, self.colors['text'], panel_rect, 2)

        # Draw text information
        y_offset = panel_y + 10
        line_height = 20

        info_lines = [
            f"Robot State: {self.additional_info.get('robot_state', 'UNKNOWN')}",
            f"Target: {self.additional_info.get('target_object', 'None')}",
            f"Total Objects: {self.additional_info.get('total_objects', 0)}",
            f"LIDAR Points: {self.additional_info.get('lidar_points', 0)}",
            "",
            "Controls:",
            "Mouse: Pan view",
            "Scroll: Zoom",
            "Ctrl+C: Clear map"
        ]

        for line in info_lines:
            if line:  # Skip empty lines
                text = self.font_small.render(line, True, self.colors['ui_text'])
                self.screen.blit(text, (panel_x + 10, y_offset))
            y_offset += line_height

    def _world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        screen_x = (world_x - self.view_center_x) * self.scale + self.window_size[0] / 2
        screen_y = (-world_y - (-self.view_center_y)) * self.scale + self.window_size[1] / 2
        return (int(screen_x), int(screen_y))

    def _screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates to world coordinates"""
        world_x = (screen_x - self.window_size[0] / 2) / self.scale + self.view_center_x
        world_y = -((screen_y - self.window_size[1] / 2) / self.scale - self.view_center_y)
        return (world_x, world_y)

    def _is_visible(self, screen_pos):
        """Check if screen position is visible"""
        return (-50 <= screen_pos[0] <= self.window_size[0] + 50 and
                -50 <= screen_pos[1] <= self.window_size[1] + 50)

    def _zoom_in(self, mouse_pos):
        """Zoom in at mouse position"""
        old_scale = self.scale
        self.scale = min(self.scale * 1.2, self.max_scale)
        self._adjust_view_center_for_zoom(mouse_pos, old_scale)

    def _zoom_out(self, mouse_pos):
        """Zoom out at mouse position"""
        old_scale = self.scale
        self.scale = max(self.scale / 1.2, self.min_scale)
        self._adjust_view_center_for_zoom(mouse_pos, old_scale)

    def _adjust_view_center_for_zoom(self, mouse_pos, old_scale):
        """Adjust view center so zoom happens at mouse position"""
        world_pos_before = self._screen_to_world(mouse_pos[0], mouse_pos[1])

        # Update scale (already done in zoom methods)

        world_pos_after = self._screen_to_world(mouse_pos[0], mouse_pos[1])

        # Adjust view center to keep mouse world position constant
        self.view_center_x += world_pos_before[0] - world_pos_after[0]
        self.view_center_y += world_pos_before[1] - world_pos_after[1]

    def _pan_view(self, mouse_pos):
        """Pan the view based on mouse movement"""
        dx = mouse_pos[0] - self.last_mouse_pos[0]
        dy = mouse_pos[1] - self.last_mouse_pos[1]

        self.view_center_x -= dx / self.scale
        self.view_center_y += dy / self.scale  # Flip Y axis

        self.last_mouse_pos = mouse_pos