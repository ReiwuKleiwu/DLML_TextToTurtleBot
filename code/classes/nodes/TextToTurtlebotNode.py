from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.controllers.MapState import MapState
from classes.behaviors.obstacle_avoidance.SimpleObstacleAvoidance import SimpleObstacleAvoidance
from classes.behaviors.exploration.RandomExploration import RandomExploration
from classes.behaviors.target_navigation.SimpleTargetNavigation import SimpleTargetNavigation
from classes.sensors.CameraHandler import CameraHandler
from classes.sensors.IRHandler import IRHandler
from classes.sensors.LIDARHandler import LIDARHandler
from classes.topics.TFSubscriber import TFSubscriber
from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator
from classes.utils.TwistWrapper import TwistWrapper

class TextToTurtlebotNode(Node):
    def __init__(self, namespace: str = '', use_turtlebot_sim: bool = False):
        super().__init__('TextToTurtlebotNode', namespace=namespace)

        self.bridge = CvBridge()

        # Initialize TurtleBot state machine and map state
        self.state_machine = StateMachine()
        self.map_state = MapState()

        # Initialize Twist and CMD-Publisher
        self.twist = TwistWrapper(use_stamped=use_turtlebot_sim)
        
        # Use TwistStamped for simulator, Twist for physical robot
        msg_type = TwistStamped if use_turtlebot_sim else Twist
        self.cmd_publisher = self.create_publisher(msg_type, f"/cmd_vel", 10)

        # Initialize Obstacle-Avoider as well as Explorer
        self.explorer = RandomExploration(self.twist, self.cmd_publisher)
        self.obstacle_avoider  = SimpleObstacleAvoidance(self.state_machine, self.twist, self.cmd_publisher)
        self.target_navigator = SimpleTargetNavigation(self.state_machine, self.twist, self.cmd_publisher)

        # Initialize TF subscriber
        self.tf_subscriber = TFSubscriber(self, base_link_frame="base_link")
        
        # Initialize Object Position Estimator
        self.position_estimator = ObjectPositionEstimator(
            tf_subscriber=self.tf_subscriber,
            camera_frame="oakd_rgb_camera_optical_frame"  # Adjust frame name as needed
        )

        # Initialize Sensor Handlers with position estimator and map state
        self.camera_handler = CameraHandler(
            bridge=self.bridge, 
            state_machine=self.state_machine,
            map_state=self.map_state,
            position_estimator=self.position_estimator
        )
        self.lidar_handler = LIDARHandler(
            state_machine=self.state_machine,
            position_estimator=self.position_estimator
        )
        self.ir_handler = IRHandler(self.state_machine)

        # Register Sensor Handlers
        self.camera_subscription = self.create_subscription(
            Image,
            f"/oakd/rgb/preview/image_raw",
            self.camera_handler.handle,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            f"/scan",
            self.lidar_handler.handle,
            10
        )

        # TODO: Add subscriptions for IR and Bumper

    def find_target(self, target: str):
        self.state_machine.push_state(
            TurtleBotState.EXPLORE,
            TurtleBotStateSource.USER,
            {"target_object": target}
        )
    
    def get_detected_objects(self, object_class: str = None):
        """
        Get detected objects from the map state.
        
        Args:
            object_class: Specific class to filter by (None for all objects)
            
        Returns:
            List of detected objects or dictionary of all objects by class
        """
        if object_class:
            return self.map_state.get_objects_by_class(object_class)
        else:
            return self.map_state.get_all_detected_objects()
    
    def get_nearest_object(self, object_class: str, max_age_seconds: float = 60.0):
        """
        Get the nearest detected object of a specific class to the robot's current position.
        
        Args:
            object_class: Class of object to search for
            max_age_seconds: Maximum age of detection to consider
            
        Returns:
            Dictionary with object info or None if not found
        """
        robot_position = self.tf_subscriber.get_position_2d()
        if robot_position is None:
            return None
            
        return self.map_state.get_nearest_object(object_class, robot_position, max_age_seconds)
    
    def get_objects_in_radius(self, radius: float, object_class: str = None, max_age_seconds: float = 60.0):
        """
        Get all objects within a certain radius of the robot's current position.
        
        Args:
            radius: Search radius in meters
            object_class: Specific object class to search for (None for all)
            max_age_seconds: Maximum age of detection to consider
            
        Returns:
            List of objects within radius
        """
        robot_position = self.tf_subscriber.get_position_2d()
        if robot_position is None:
            return []
            
        return self.map_state.get_objects_in_radius(robot_position, radius, object_class, max_age_seconds)
    
    def get_detection_statistics(self):
        """Get statistics about detected objects."""
        return self.map_state.get_detection_statistics()
    
    def cleanup_old_detections(self, max_age_seconds: float = 300.0):
        """Remove old object detections from the map."""
        self.map_state.remove_old_detections(max_age_seconds)

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