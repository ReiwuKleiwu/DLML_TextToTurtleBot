import pygame
import math
import threading
import time

class InteractiveMapVisualizer:
    def __init__(self, window_size=(1000, 800), initial_world_bounds=(-5.0, -5.0, 5.0, 5.0)):
        """
        Initialize the interactive map visualizer using pygame

        Args:
            window_size: (width, height) of the visualization window
            initial_world_bounds: (min_x, min_y, max_x, max_y) initial world coordinates to display
        """
        pygame.init()

        self.window_size = window_size
        self.screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("TurtleBot Interactive World Map")

        # Map view parameters
        self.view_center_x = 0.0  # Center of view in world coordinates
        self.view_center_y = 0.0
        self.scale = 50.0  # Pixels per meter (initial zoom)
        self.min_scale = 10.0
        self.max_scale = 200.0

        # Interaction state
        self.dragging = False
        self.last_mouse_pos = (0, 0)

        # Colors (RGB format for pygame)
        self.colors = {
            'background': (50, 50, 50),
            'grid': (80, 80, 80),
            'grid_major': (120, 120, 120),
            'dock': (255, 255, 0),           # Yellow
            'turtlebot': (0, 100, 255),      # Blue
            'target_object': (255, 0, 0),    # Red
            'other_object': (0, 255, 0),     # Green
            'text': (255, 255, 255),         # White
            'trajectory': (255, 128, 128),   # Light red
            'ui_panel': (30, 30, 30),        # Dark gray
            'ui_text': (200, 200, 200),      # Light gray
            'lidar_scan': (100, 150, 255),   # Light blue
            'lidar_obstacle': (255, 150, 100), # Orange
            'lidar_wall': (200, 100, 200)    # Purple
        }

        # Fonts
        self.font_small = pygame.font.Font(None, 20)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_large = pygame.font.Font(None, 32)

        # Data storage
        self.trajectory_points = []
        self.max_trajectory_points = 500
        self.world_object_map = {}
        self.turtlebot_position = None
        self.turtlebot_orientation = None
        self.target_object_class = None
        self.additional_info = {}

        # LIDAR data storage
        self.lidar_scan_points = []
        self.lidar_obstacle_points = []
        self.lidar_walls = []  # Processed wall segments
        self.show_lidar_scan = True
        self.show_lidar_obstacles = True
        self.show_lidar_walls = True

        # LIDAR interpolation system - SIMPLIFIED FOR DEBUGGING
        self.previous_lidar_scan_points = []
        self.previous_lidar_obstacle_points = []
        self.lidar_interpolation_progress = 1.0  # 0.0 = old position, 1.0 = new position
        self.lidar_interpolation_speed = 0.35  # How fast to interpolate (per frame) - increased for responsiveness
        self.lidar_interpolation_enabled = False  # DISABLE for debugging
        self.lidar_update_throttle = 0.05  # Reduce throttle to 50ms to match 20Hz timer
        self.last_lidar_update_time = 0
        self.lidar_frame_counter = 0

        # Callbacks
        self.clear_map_callback = None

        # Threading
        self.running = True
        self.update_thread = None

    def world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        # Translate to view center, then scale, then translate to screen center
        screen_x = (world_x - self.view_center_x) * self.scale + self.window_size[0] / 2
        screen_y = (world_y - self.view_center_y) * self.scale + self.window_size[1] / 2
        # Flip Y axis (pygame has origin at top-left, we want bottom-left)
        screen_y = self.window_size[1] - screen_y
        return int(screen_x), int(screen_y)

    def screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates to world coordinates"""
        # Flip Y axis first
        screen_y = self.window_size[1] - screen_y
        # Translate to center, scale, then translate to view center
        world_x = (screen_x - self.window_size[0] / 2) / self.scale + self.view_center_x
        world_y = (screen_y - self.window_size[1] / 2) / self.scale + self.view_center_y
        return world_x, world_y

    def handle_events(self):
        """Handle pygame events (mouse, keyboard)"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    self.dragging = True
                    self.last_mouse_pos = event.pos

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left mouse button
                    self.dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.dragging:
                    # Calculate drag delta in world coordinates
                    dx = (event.pos[0] - self.last_mouse_pos[0]) / self.scale
                    dy = -(event.pos[1] - self.last_mouse_pos[1]) / self.scale  # Flip Y

                    # Update view center
                    self.view_center_x -= dx
                    self.view_center_y -= dy

                    self.last_mouse_pos = event.pos

            elif event.type == pygame.MOUSEWHEEL:
                # Zoom with mouse wheel
                mouse_pos = pygame.mouse.get_pos()
                world_pos_before = self.screen_to_world(mouse_pos[0], mouse_pos[1])

                # Update scale
                zoom_factor = 1.1 if event.y > 0 else 0.9
                self.scale = max(self.min_scale, min(self.max_scale, self.scale * zoom_factor))

                # Adjust view center to keep mouse position fixed
                world_pos_after = self.screen_to_world(mouse_pos[0], mouse_pos[1])
                self.view_center_x += world_pos_before[0] - world_pos_after[0]
                self.view_center_y += world_pos_before[1] - world_pos_after[1]

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_HOME or event.key == pygame.K_r:
                    # Reset view to origin
                    self.view_center_x = 0.0
                    self.view_center_y = 0.0
                    self.scale = 50.0
                elif event.key == pygame.K_t:
                    # Center on TurtleBot
                    if self.turtlebot_position:
                        self.view_center_x = self.turtlebot_position[0]
                        self.view_center_y = self.turtlebot_position[1]
                elif event.key == pygame.K_l:
                    # Toggle LIDAR scan points
                    self.show_lidar_scan = not self.show_lidar_scan
                elif event.key == pygame.K_o:
                    # Toggle LIDAR obstacles
                    self.show_lidar_obstacles = not self.show_lidar_obstacles
                elif event.key == pygame.K_w:
                    # Toggle LIDAR walls
                    self.show_lidar_walls = not self.show_lidar_walls
                elif event.key == pygame.K_c:
                    # Clear persistent map (will be handled by callback)
                    if hasattr(self, 'clear_map_callback') and self.clear_map_callback:
                        self.clear_map_callback()
                elif event.key == pygame.K_i:
                    # Toggle LIDAR interpolation
                    self.lidar_interpolation_enabled = not self.lidar_interpolation_enabled

    def draw_grid(self):
        """Draw coordinate grid"""
        # Calculate visible world bounds
        margin = 2.0  # Extra margin in world coordinates
        top_left = self.screen_to_world(-margin * self.scale, -margin * self.scale)
        bottom_right = self.screen_to_world(self.window_size[0] + margin * self.scale,
                                          self.window_size[1] + margin * self.scale)

        # Draw grid lines
        grid_spacing = 1.0  # 1 meter grid
        if self.scale < 25:
            grid_spacing = 2.0  # Larger spacing when zoomed out
        elif self.scale > 100:
            grid_spacing = 0.5  # Smaller spacing when zoomed in

        # Vertical lines
        start_x = int(top_left[0] / grid_spacing) * grid_spacing
        for x in [start_x + i * grid_spacing for i in range(int((bottom_right[0] - start_x) / grid_spacing) + 2)]:
            color = self.colors['grid_major'] if abs(x) < 0.01 else self.colors['grid']  # Highlight axes
            thickness = 2 if abs(x) < 0.01 else 1

            start_pos = self.world_to_screen(x, top_left[1])
            end_pos = self.world_to_screen(x, bottom_right[1])
            if 0 <= start_pos[0] <= self.window_size[0] or 0 <= end_pos[0] <= self.window_size[0]:
                pygame.draw.line(self.screen, color, start_pos, end_pos, thickness)

        # Horizontal lines
        start_y = int(top_left[1] / grid_spacing) * grid_spacing
        for y in [start_y + i * grid_spacing for i in range(int((bottom_right[1] - start_y) / grid_spacing) + 2)]:
            color = self.colors['grid_major'] if abs(y) < 0.01 else self.colors['grid']  # Highlight axes
            thickness = 2 if abs(y) < 0.01 else 1

            start_pos = self.world_to_screen(top_left[0], y)
            end_pos = self.world_to_screen(bottom_right[0], y)
            if 0 <= start_pos[1] <= self.window_size[1] or 0 <= end_pos[1] <= self.window_size[1]:
                pygame.draw.line(self.screen, color, start_pos, end_pos, thickness)

    def draw_dock(self):
        """Draw dock at origin (0, 0)"""
        dock_pos = self.world_to_screen(0.0, 0.0)

        # Only draw if visible
        if (-50 <= dock_pos[0] <= self.window_size[0] + 50 and
            -50 <= dock_pos[1] <= self.window_size[1] + 50):

            radius = max(8, int(self.scale * 0.3))
            pygame.draw.circle(self.screen, self.colors['dock'], dock_pos, radius)
            pygame.draw.circle(self.screen, self.colors['text'], dock_pos, radius, 2)

            # Label
            if self.scale > 30:  # Only show label when zoomed in enough
                text = self.font_medium.render("DOCK", True, self.colors['dock'])
                text_rect = text.get_rect(center=(dock_pos[0], dock_pos[1] - radius - 15))
                self.screen.blit(text, text_rect)

    def draw_turtlebot(self):
        """Draw TurtleBot with trajectory"""
        if not self.turtlebot_position:
            return

        x, y, z = self.turtlebot_position
        pos = self.world_to_screen(x, y)

        # Only draw if visible
        if (-100 <= pos[0] <= self.window_size[0] + 100 and
            -100 <= pos[1] <= self.window_size[1] + 100):

            # Add to trajectory
            self.trajectory_points.append((x, y))
            if len(self.trajectory_points) > self.max_trajectory_points:
                self.trajectory_points.pop(0)

            # Draw trajectory
            if len(self.trajectory_points) > 1 and self.scale > 20:
                screen_points = [self.world_to_screen(px, py) for px, py in self.trajectory_points[-100:]]  # Limit for performance
                valid_points = [(px, py) for px, py in screen_points
                              if -50 <= px <= self.window_size[0] + 50 and -50 <= py <= self.window_size[1] + 50]
                if len(valid_points) > 1:
                    pygame.draw.lines(self.screen, self.colors['trajectory'], False, valid_points, 2)

            # Draw robot body
            radius = max(10, int(self.scale * 0.2))
            pygame.draw.circle(self.screen, self.colors['turtlebot'], pos, radius)
            pygame.draw.circle(self.screen, self.colors['text'], pos, radius, 2)

            # Draw orientation arrow
            if self.turtlebot_orientation and self.scale > 25:
                qx, qy, qz, qw = self.turtlebot_orientation
                yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

                arrow_length = radius + 15
                end_x = pos[0] + int(arrow_length * math.cos(yaw))
                end_y = pos[1] - int(arrow_length * math.sin(yaw))  # Negative because pygame Y is flipped
                pygame.draw.line(self.screen, self.colors['turtlebot'], pos, (end_x, end_y), 3)

                # Arrow head
                head_size = 5
                head_angle1 = yaw + 2.5
                head_angle2 = yaw - 2.5
                head1 = (end_x - int(head_size * math.cos(head_angle1)),
                         end_y + int(head_size * math.sin(head_angle1)))
                head2 = (end_x - int(head_size * math.cos(head_angle2)),
                         end_y + int(head_size * math.sin(head_angle2)))
                pygame.draw.polygon(self.screen, self.colors['turtlebot'], [(end_x, end_y), head1, head2])

            # Position label
            if self.scale > 40:
                pos_text = f"({x:.1f}, {y:.1f})"
                text = self.font_small.render(pos_text, True, self.colors['text'])
                text_rect = text.get_rect(center=(pos[0], pos[1] + radius + 15))
                self.screen.blit(text, text_rect)

    def draw_objects(self):
        """Draw detected objects"""
        if not self.world_object_map:
            return

        for object_class, objects in self.world_object_map.items():
            for obj_info in objects:
                if not obj_info.get('world_coords'):
                    continue

                wx, wy, wz = obj_info['world_coords']
                pos = self.world_to_screen(wx, wy)

                # Only draw if visible
                if (-50 <= pos[0] <= self.window_size[0] + 50 and
                    -50 <= pos[1] <= self.window_size[1] + 50):

                    # Choose appearance based on object type
                    is_target = (object_class == self.target_object_class)
                    is_selected = obj_info.get('is_selected_target', False)

                    if is_selected:
                        color = self.colors['target_object']
                        radius = max(6, int(self.scale * 0.15))
                        border = True
                    elif is_target:
                        color = (255, 165, 0)  # Orange for other targets of same class
                        radius = max(5, int(self.scale * 0.12))
                        border = True
                    else:
                        color = self.colors['other_object']
                        radius = max(4, int(self.scale * 0.1))
                        border = False

                    # Draw object marker
                    pygame.draw.circle(self.screen, color, pos, radius)
                    if border:
                        pygame.draw.circle(self.screen, self.colors['text'], pos, radius + 1, 2)

                    # Object label (only when zoomed in enough)
                    if self.scale > 35:
                        obj_id = obj_info.get('object_id', '')
                        label = f"{object_class}#{obj_id}" if obj_id != '' else object_class

                        text = self.font_small.render(label, True, self.colors['text'])
                        text_rect = text.get_rect(center=(pos[0], pos[1] - radius - 10))
                        self.screen.blit(text, text_rect)

    def process_lidar_walls(self, scan_points):
        """Process LIDAR scan points into wall segments using line clustering"""
        if len(scan_points) < 3:
            return []

        walls = []
        current_wall = []
        max_point_distance = 0.3  # Maximum distance between consecutive points in a wall
        min_wall_length = 0.5     # Minimum wall length to consider

        # Sort points by angle from robot (assuming robot at origin of scan)
        if not self.turtlebot_position:
            return []

        robot_x, robot_y = self.turtlebot_position[0], self.turtlebot_position[1]

        # Calculate angles and distances from robot
        point_data = []
        for px, py in scan_points:
            angle = math.atan2(py - robot_y, px - robot_x)
            distance = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            point_data.append((angle, distance, px, py))

        # Sort by angle
        point_data.sort(key=lambda x: x[0])

        # Group points into wall segments
        for i, (angle, dist, px, py) in enumerate(point_data):
            if not current_wall:
                current_wall = [(px, py)]
            else:
                # Check distance to last point in current wall
                last_px, last_py = current_wall[-1]
                point_dist = math.sqrt((px - last_px)**2 + (py - last_py)**2)

                if point_dist <= max_point_distance:
                    current_wall.append((px, py))
                else:
                    # End current wall and start new one
                    if len(current_wall) >= 2:
                        # Check if wall is long enough
                        wall_length = sum(
                            math.sqrt((current_wall[j+1][0] - current_wall[j][0])**2 +
                                     (current_wall[j+1][1] - current_wall[j][1])**2)
                            for j in range(len(current_wall) - 1)
                        )
                        if wall_length >= min_wall_length:
                            walls.append(current_wall[:])

                    current_wall = [(px, py)]

        # Don't forget the last wall
        if len(current_wall) >= 2:
            wall_length = sum(
                math.sqrt((current_wall[j+1][0] - current_wall[j][0])**2 +
                         (current_wall[j+1][1] - current_wall[j][1])**2)
                for j in range(len(current_wall) - 1)
            )
            if wall_length >= min_wall_length:
                walls.append(current_wall)

        return walls

    def _lidar_points_changed_significantly(self, new_scan_points, new_obstacle_points):
        """Check if LIDAR points have changed significantly enough to warrant update"""
        # If no previous data, always update
        if not self.lidar_scan_points and not self.lidar_obstacle_points:
            return True

        # If different number of points, update
        if (len(new_scan_points) != len(self.lidar_scan_points) or
            len(new_obstacle_points) != len(self.lidar_obstacle_points)):
            return True

        # Check if points have moved significantly (sample every 10th point for performance)
        threshold = 0.05  # 5cm threshold

        # Check scan points (sample)
        for i in range(0, min(len(new_scan_points), len(self.lidar_scan_points)), 10):
            if len(new_scan_points) > i and len(self.lidar_scan_points) > i:
                new_x, new_y = new_scan_points[i]
                old_x, old_y = self.lidar_scan_points[i]
                distance = ((new_x - old_x)**2 + (new_y - old_y)**2)**0.5
                if distance > threshold:
                    return True

        return False

    def interpolate_lidar_points(self, old_points, new_points, progress):
        """Interpolate between old and new LIDAR points"""
        if not old_points or not new_points or progress >= 1.0:
            return new_points

        if progress <= 0.0:
            return old_points

        # Handle different lengths more intelligently
        interpolated_points = []

        # If both lists have the same length, do direct interpolation
        if len(old_points) == len(new_points):
            for i in range(len(old_points)):
                old_x, old_y = old_points[i]
                new_x, new_y = new_points[i]

                # Linear interpolation
                interp_x = old_x + (new_x - old_x) * progress
                interp_y = old_y + (new_y - old_y) * progress

                interpolated_points.append((interp_x, interp_y))
        else:
            # Different lengths - use a weighted blend approach
            # Fade from old points (with decreasing opacity) to new points (with increasing opacity)

            # Add old points with decreasing weight
            for old_x, old_y in old_points:
                if progress < 0.5:  # Show old points in first half of transition
                    weight = 1.0 - (progress * 2.0)  # 1.0 to 0.0
                    if weight > 0.1:  # Only show if weight is significant
                        interpolated_points.append((old_x, old_y))

            # Add new points with increasing weight
            for new_x, new_y in new_points:
                if progress > 0.5:  # Show new points in second half of transition
                    weight = (progress - 0.5) * 2.0  # 0.0 to 1.0
                    if weight > 0.1:  # Only show if weight is significant
                        interpolated_points.append((new_x, new_y))

            # If we're in the middle transition, blend both sets
            if 0.3 <= progress <= 0.7:
                # Sample from both sets
                old_sample = old_points[::2] if old_points else []  # Every other point
                new_sample = new_points[::2] if new_points else []  # Every other point
                interpolated_points.extend(old_sample + new_sample)

        return interpolated_points

    def update_lidar_interpolation(self):
        """Update LIDAR interpolation progress"""
        if not self.lidar_interpolation_enabled:
            return

        if self.lidar_interpolation_progress < 1.0:
            old_progress = self.lidar_interpolation_progress
            self.lidar_interpolation_progress = min(1.0,
                self.lidar_interpolation_progress + self.lidar_interpolation_speed)
            # DEBUG: Only print occasionally to avoid spam
            if old_progress < 0.5 and self.lidar_interpolation_progress >= 0.5:
                print(f"[LIDAR DEBUG] Interpolation progress: {self.lidar_interpolation_progress:.2f}")

    def draw_lidar_data(self):
        """Draw LIDAR scan data - SIMPLIFIED for debugging"""
        # Draw scan points directly without interpolation
        if self.show_lidar_scan and self.lidar_scan_points:
            points_drawn = 0
            for wx, wy in self.lidar_scan_points:
                pos = self.world_to_screen(wx, wy)
                if (-10 <= pos[0] <= self.window_size[0] + 10 and
                    -10 <= pos[1] <= self.window_size[1] + 10):
                    pygame.draw.circle(self.screen, self.colors['lidar_scan'], pos, 1)
                    points_drawn += 1


        # Draw obstacle points directly without interpolation
        if self.show_lidar_obstacles and self.lidar_obstacle_points:
            obstacles_drawn = 0
            for wx, wy in self.lidar_obstacle_points:
                pos = self.world_to_screen(wx, wy)
                if (-10 <= pos[0] <= self.window_size[0] + 10 and
                    -10 <= pos[1] <= self.window_size[1] + 10):
                    pygame.draw.circle(self.screen, self.colors['lidar_obstacle'], pos, 2)
                    obstacles_drawn += 1


        # Draw walls (lines connecting wall points)
        if self.show_lidar_walls and self.lidar_walls:
            for wall in self.lidar_walls:
                if len(wall) < 2:
                    continue

                screen_points = []
                for wx, wy in wall:
                    pos = self.world_to_screen(wx, wy)
                    screen_points.append(pos)

                # Only draw if at least part of the wall is visible
                visible = any(-50 <= pos[0] <= self.window_size[0] + 50 and
                             -50 <= pos[1] <= self.window_size[1] + 50
                             for pos in screen_points)

                if visible and len(screen_points) >= 2:
                    pygame.draw.lines(self.screen, self.colors['lidar_wall'], False, screen_points, 2)

    def draw_ui_panel(self):
        """Draw information panel"""
        panel_height = 140  # Increased for LIDAR status
        panel_rect = pygame.Rect(0, self.window_size[1] - panel_height, self.window_size[0], panel_height)
        pygame.draw.rect(self.screen, self.colors['ui_panel'], panel_rect)
        pygame.draw.rect(self.screen, self.colors['text'], panel_rect, 2)

        y_offset = self.window_size[1] - panel_height + 10
        x_offset = 10

        # Object counts
        if 'object_count' in self.additional_info and self.additional_info['object_count']:
            count_text = "Objects: " + ", ".join([f"{obj_class}: {count}"
                                                 for obj_class, count in self.additional_info['object_count'].items()])
            text = self.font_small.render(count_text, True, self.colors['ui_text'])
            self.screen.blit(text, (x_offset, y_offset))
            y_offset += 20

        # Robot state and target
        if 'robot_state' in self.additional_info:
            state_text = f"State: {self.additional_info['robot_state']}"
            text = self.font_small.render(state_text, True, self.colors['ui_text'])
            self.screen.blit(text, (x_offset, y_offset))

        if 'target_object' in self.additional_info and self.additional_info['target_object']:
            target_text = f"Target: {self.additional_info['target_object']}"
            text = self.font_small.render(target_text, True, self.colors['target_object'])
            self.screen.blit(text, (x_offset + 150, y_offset))

        y_offset += 20

        # View info
        view_text = f"View: ({self.view_center_x:.1f}, {self.view_center_y:.1f}) Scale: {self.scale:.1f}px/m"
        text = self.font_small.render(view_text, True, self.colors['ui_text'])
        self.screen.blit(text, (x_offset, y_offset))

        y_offset += 20

        # Controls
        controls_text = "Controls: Drag=pan, Scroll=zoom, R=reset, T=center robot, L=scan, O=obstacles, W=walls, C=clear map, I=interpolation"
        text = self.font_small.render(controls_text, True, self.colors['ui_text'])
        self.screen.blit(text, (x_offset, y_offset))

        # LIDAR status
        y_offset += 20
        interp_status = 'ON' if self.lidar_interpolation_enabled else 'OFF'
        lidar_status = f"LIDAR: Scan={'ON' if self.show_lidar_scan else 'OFF'} | Obstacles={'ON' if self.show_lidar_obstacles else 'OFF'} | Walls={'ON' if self.show_lidar_walls else 'OFF'} | Interpolation={interp_status}"
        text = self.font_small.render(lidar_status, True, self.colors['lidar_scan'])
        self.screen.blit(text, (x_offset, y_offset))

    def update_data(self, turtlebot_position, turtlebot_orientation, world_object_map,
                   target_object_class=None, additional_info=None, lidar_data=None):
        """Update data for visualization (thread-safe)"""
        import time

        self.turtlebot_position = turtlebot_position
        self.turtlebot_orientation = turtlebot_orientation
        self.world_object_map = world_object_map or {}
        self.target_object_class = target_object_class
        self.additional_info = additional_info or {}

        # Update LIDAR data - SIMPLIFIED for debugging
        if lidar_data:
            current_time = time.time()
            new_scan_points = lidar_data.get('scan_points', [])
            new_obstacle_points = lidar_data.get('obstacle_points', [])

            # Update LIDAR data directly
            if new_scan_points != self.lidar_scan_points or new_obstacle_points != self.lidar_obstacle_points:
                self.lidar_scan_points = new_scan_points
                self.lidar_obstacle_points = new_obstacle_points
                self.last_lidar_update_time = current_time

            # Process walls from current scan points
            self.lidar_walls = self.process_lidar_walls(self.lidar_scan_points)

    def run_visualization_loop(self):
        """Main visualization loop (run in separate thread)"""
        clock = pygame.time.Clock()

        while self.running:
            # Handle events
            self.handle_events()

            # Clear screen
            self.screen.fill(self.colors['background'])

            # Draw everything
            self.draw_grid()
            self.draw_lidar_data()  # Draw LIDAR data behind other objects
            self.draw_dock()
            self.draw_turtlebot()
            self.draw_objects()
            self.draw_ui_panel()

            # Update display
            pygame.display.flip()
            clock.tick(30)  # 30 FPS

        pygame.quit()

    def start(self):
        """Start the visualization in a separate thread"""
        if not self.update_thread or not self.update_thread.is_alive():
            self.running = True
            self.update_thread = threading.Thread(target=self.run_visualization_loop)
            self.update_thread.daemon = True
            self.update_thread.start()

    def stop(self):
        """Stop the visualization"""
        self.running = False
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)

    def is_running(self):
        """Check if visualization is running"""
        return self.running and self.update_thread and self.update_thread.is_alive()

    def set_clear_map_callback(self, callback):
        """Set callback function for clearing the persistent map"""
        self.clear_map_callback = callback