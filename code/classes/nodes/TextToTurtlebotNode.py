from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.behaviors.obstacle_avoidance.SimpleObstacleAvoidance import SimpleObstacleAvoidance
from classes.behaviors.exploration.RandomExploration import RandomExploration
from classes.behaviors.target_navigation.SimpleTargetNavigation import SimpleTargetNavigation
from classes.sensors.CameraHandler import CameraHandler
from classes.sensors.DepthCameraHandler import DepthCameraHandler
from classes.sensors.IRHandler import IRHandler
from classes.sensors.LIDARHandler import LIDARHandler
from classes.topics.TFSubscriber import TFSubscriber
from classes.utils.TwistWrapper import TwistWrapper
from classes.services.MapService import MapService
from classes.services.VisualizationService import VisualizationService

class TextToTurtlebotNode(Node):
    def __init__(self, namespace: str = '', use_turtlebot_sim: bool = False):
        super().__init__('TextToTurtlebotNode', namespace=namespace)

        self.bridge = CvBridge()

        # Initialize TurtleBot state machine
        self.state_machine = StateMachine()

        # Initialize Twist and CMD-Publisher
        self.twist = TwistWrapper(use_stamped=use_turtlebot_sim)
        
        # Use TwistStamped for simulator, Twist for physical robot
        msg_type = TwistStamped if use_turtlebot_sim else Twist
        self.cmd_publisher = self.create_publisher(msg_type, f"/cmd_vel", 10)

        # Initialize Obstacle-Avoider as well as Explorer
        self.explorer = RandomExploration(self.twist, self.cmd_publisher)
        self.obstacle_avoider  = SimpleObstacleAvoidance(self.state_machine, self.twist, self.cmd_publisher)
        self.target_navigator = SimpleTargetNavigation(self.state_machine, self.twist, self.cmd_publisher)


        # Initialize Sensor Handlers
        self.depth_camera_handler = DepthCameraHandler(self.bridge, self.state_machine)
        self.depth_camera_handler.set_logger(self.get_logger())
        self.camera_handler = CameraHandler(self.bridge, self.state_machine, self.depth_camera_handler)
        self.lidar_handler = LIDARHandler(self.state_machine)
        self.ir_handler = IRHandler(self.state_machine)
        self.tf_subscriber = TFSubscriber(self, base_link_frame="base_link")

        # Share TF buffer with depth camera handler for world coordinate transformations
        self.depth_camera_handler.set_tf_buffer(self.tf_subscriber.tf_buffer)

        # Share TF buffer with LIDAR handler for coordinate transformations
        self.lidar_handler.set_tf_buffer(self.tf_subscriber.tf_buffer)

        # Initialize map service for persistent object tracking
        self.map_service = MapService()

        # Initialize visualization service
        self.visualization_service = VisualizationService(
            map_service=self.map_service,
            tf_subscriber=self.tf_subscriber,
            state_machine=self.state_machine,
            window_size=(1000, 800),
            world_bounds=(-5.0, -5.0, 5.0, 5.0)
        )

        # Set callback for coordinate updates from depth handler
        self.depth_camera_handler.set_coordinate_callback(self.map_service.update_world_object_map)

        # Set callback for LIDAR scan updates
        self.lidar_handler.set_scan_callback(self.visualization_service.update_lidar_data)

        # Add callback to share map updates with camera handler
        self.map_service.add_coordinate_update_callback(self._on_map_updated)

        # Create timer for map visualization updates (20 Hz)
        self.map_update_timer = self.create_timer(0.05, self._update_visualization_timer)

        # Register Sensor Handlers
        self.camera_subscription = self.create_subscription(
            Image,
            f"/oakd/rgb/preview/image_raw",
            self.camera_handler.handle,
            10
        )

        self.depth_camera_subscription = self.create_subscription(
            Image,
            f"/oakd/rgb/preview/depth",
            self.depth_camera_handler.handle,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            f"/scan",
            self.lidar_handler.handle,
            10
        )

        self.depth_camera_info_subscription = self.create_subscription(
            CameraInfo,
            f"/oakd/rgb/preview/camera_info",
            self.depth_camera_handler.set_camera_intrinsics,
            10
        )

        # TODO: Add subscriptions for IR and Bumper

    def _on_map_updated(self, world_object_map):
        """Callback when map service updates the world object map"""
        # Share the updated world coordinates with the camera handler for display
        self.camera_handler.set_world_coordinates(world_object_map)

    def _update_visualization_timer(self):
        """Timer callback to update map visualization"""
        self.visualization_service.update_map_visualization(self.get_clock())

    # Delegation methods for backward compatibility
    def get_world_object_map(self):
        """Get the current world object map"""
        return self.map_service.get_world_object_map()

    def get_objects_by_type(self, object_type):
        """Get all objects of a specific type with their world coordinates"""
        return self.map_service.get_objects_by_type(object_type)

    def find_target(self, target: str):
        self.state_machine.push_state(
            TurtleBotState.EXPLORE,
            TurtleBotStateSource.USER,
            {"target_object": target}
        )

    def handle_state(self):
        current_state = self.state_machine.get_current_state()

        if current_state is None:
            raise Exception('[ERROR]: State cannot be None.')

        match current_state.value:
            case TurtleBotState.EXPLORE:
                # print('[INFO]: Exploring...')
                self.explorer.execute()
            case TurtleBotState.AVOID_OBSTACLE:
                # print('[INFO]: Avoiding obstacle...')
                self.obstacle_avoider.execute()
            case TurtleBotState.OBJECT_FOUND:
                #print('[INFO]: Target Object found...')
                self.target_navigator.execute()
            case TurtleBotState.IDLE:
                print('[INFO]: Idling...')
            case _:
                print(f'[INFO]: Unknown state: {current_state.value}')

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'visualization_service'):
            self.visualization_service.stop()