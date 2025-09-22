"""ROS node wiring the clean-architecture components together."""
from __future__ import annotations

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from code.application.behaviors.exploration import RandomExploration
from code.application.behaviors.navigation_task import NavigationTask
from code.application.behaviors.obstacle_avoidance import SimpleObstacleAvoidance
from code.application.behaviors.target_navigation import SimpleTargetNavigation
from code.application.llm_interface.control_api import RobotControlAPI
from code.application.controllers.navigation_controller import TargetSearchNavigator
from code.application.controllers.obstacle_controller import ObstacleController
from code.application.controllers.vision_controller import VisionController
from code.application.runtime.orchestrator import TurtleBotOrchestrator
from code.application.services.map_tracker import MapTrackerService
from code.application.services.target_reached import TargetReachedService
from code.application.services.tf_monitor import TransformMonitor
from code.application.utils.twist_wrapper import TwistWrapper
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState
from code.core.state_machine import RobotStateMachine
from code.infrastructure.perception.camera_processor import CameraProcessor
from code.infrastructure.perception.depth_processor import DepthProcessor
from code.infrastructure.ros.navigation.slam_navigation_service import SLAMNavigationService
from code.infrastructure.ros.tf_subscriber import TFSubscriber
from code.infrastructure.sensors.ir_processor import IRProcessor
from code.infrastructure.sensors.lidar_processor import LidarProcessor
from code.infrastructure.visualization.visualization_service import VisualizationService


class TurtleBotNode(Node):
    """Primary ROS node hosting the TurtleBot orchestration."""

    def __init__(self, event_bus: EventBus, namespace: str = '', use_turtlebot_sim: bool = False) -> None:
        super().__init__('TextToTurtlebotNode', namespace=namespace)

        self._event_bus = event_bus
        self._state_machine = RobotStateMachine(self._event_bus)

        self._twist = TwistWrapper(use_stamped=use_turtlebot_sim)
        msg_type = TwistStamped if use_turtlebot_sim else Twist
        self._cmd_publisher = self.create_publisher(msg_type, '/cmd_vel', 10)

        self._map_service = MapTrackerService(self._event_bus)
        self._visualization_service = VisualizationService(self._event_bus)

        bridge = CvBridge()

        self._camera_processor = CameraProcessor(
            event_bus=self._event_bus,
            bridge=bridge,
            model_path='models',
        )
        self._depth_processor = DepthProcessor(self._event_bus, bridge)
        self._lidar_processor = LidarProcessor(self._event_bus)
        self._ir_processor = IRProcessor(self._event_bus)
        self._depth_processor.set_logger(self.get_logger())

        self._tf_subscriber = TFSubscriber(self, base_link_frame="base_link")
        self._transform_monitor = TransformMonitor(self._event_bus, self._tf_subscriber)

        self._navigation_service = SLAMNavigationService(self, self._event_bus)
        self._target_reached_service = TargetReachedService(
            event_bus=self._event_bus,
            state_machine=self._state_machine,
            reach_distance_threshold=1.75,
        )

        self._vision_controller = VisionController(
            event_bus=self._event_bus,
            state_machine=self._state_machine,
            map_tracker=self._map_service,
        )
        self._obstacle_controller = ObstacleController(self._event_bus, self._state_machine)
        
        self._target_search_navigator = TargetSearchNavigator(
            event_bus=self._event_bus,
            state_machine=self._state_machine,
            navigation_service=self._navigation_service,
            logger=self.get_logger(),
        )

        behaviours = {
            RobotState.EXPLORE: RandomExploration(self._twist, self._cmd_publisher),
            RobotState.AVOID_OBSTACLE: SimpleObstacleAvoidance(self._state_machine, self._twist, self._cmd_publisher),
            RobotState.OBJECT_FOUND: SimpleTargetNavigation(self._state_machine, self._twist, self._cmd_publisher),
            RobotState.NAVIGATE: NavigationTask(
                state_machine=self._state_machine,
                navigation_service=self._navigation_service,
                twist=self._twist,
                publisher=self._cmd_publisher,
            ),
        }
        
        self.orchestrator = TurtleBotOrchestrator(
            event_bus=self._event_bus,
            state_machine=self._state_machine,
            behaviours=behaviours,
        )

        self._depth_processor.set_tf_buffer(self._tf_subscriber.tf_buffer)
        self._lidar_processor.set_tf_buffer(self._tf_subscriber.tf_buffer)

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self._camera_processor.handle, 10)
        self.create_subscription(Image, '/oakd/rgb/preview/depth', self._depth_processor.handle, 10)
        self.create_subscription(LaserScan, '/scan', self._lidar_processor.handle, 10)
        self.create_subscription(CameraInfo, '/oakd/rgb/preview/camera_info', self._depth_processor.set_camera_intrinsics, 10)
        self.create_subscription(IrIntensityVector, '/ir_intensity', self._ir_processor.handle, 10)

        self._transform_update_timer = self.create_timer(0.1, self._transform_monitor.update_robot_transforms)

        self.control_api = RobotControlAPI(
            state_machine=self._state_machine,
            orchestrator=self.orchestrator,
            camera_processor=self._camera_processor,
            navigation_service=self._navigation_service,
            target_navigator=self._target_search_navigator,
            map_service=self._map_service,
            visualization_service=self._visualization_service,
            transform_monitor=self._transform_monitor,
            event_bus=self._event_bus,
            logger=self.get_logger(),
        )

    def request_target(self, target: str) -> None:
        self._camera_processor.set_target_object(target)
        self.orchestrator.request_target(target)

    def __del__(self):
        pass
