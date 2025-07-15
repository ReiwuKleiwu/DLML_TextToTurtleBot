from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.behaviors.obstacle_avoidance.SimpleObstacleAvoidance import SimpleObstacleAvoidance
from classes.behaviors.exploration.RandomExploration import RandomExploration
from classes.behaviors.target_navigation.SimpleTargetNavigation import SimpleTargetNavigation
from classes.sensors.CameraHandler import CameraHandler
from classes.sensors.IRHandler import IRHandler
from classes.sensors.LIDARHandler import LIDARHandler
from classes.utils.TwistWrapper import TwistWrapper

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
        self.camera_handler = CameraHandler(self.bridge, self.state_machine)
        self.lidar_handler = LIDARHandler(self.state_machine)
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