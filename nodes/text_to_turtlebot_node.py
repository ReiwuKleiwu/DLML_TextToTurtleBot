import queue
import threading
import time
from typing import List, Optional

from cv_bridge import CvBridge
from perception.camera.camera_processor import CameraProcessor
from perception.camera.depth_camera_processor import DepthCameraProcessor
from perception.detection.object_detector import ObjectDetector
from perception.detection.target_reached_detector import TargetReachedDetector
from perception.detection.target_selector import TargetSelector
from perception.tf.tf_subscriber import TFSubscriber
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import Twist, TwistStamped
from utils.twist_wrapper import TwistWrapper
from behaviours.actions.turn_around import TurnAround
from behaviours.conditions.check_lidar import CheckLidar
from behaviours.conditions.navigation_goal_idle import NavigationGoalIdle
from behaviours.user_command_executor import UserCommandExecutor
from navigation.nav2_client import Nav2Client
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import String
from events.event_bus import EventBus
from blackboard.blackboard import Blackboard
from map.map import Map
from natural_language_processing.llm_api import LLMAPI
from langchain_core.messages import BaseMessage

from perception.lidar.lidar_processor import LidarProcessor

class TextToTurtlebotNode(Node):
    def __init__(self, namespace: str = '', use_turtlebot_sim: bool = False) -> None:
        super().__init__('TextToTurtlebotNode', namespace=namespace)
        self.get_logger().info('Text to Turtlebot Node has been started.')

        self._event_bus = EventBus()
        self._blackboard = Blackboard()

        self._bridge = CvBridge()

        self._twist = TwistWrapper(use_stamped=use_turtlebot_sim)
        msg_type = TwistStamped if use_turtlebot_sim else Twist
        self._cmd_publisher = self.create_publisher(msg_type, 'cmd_vel', 10)

        self._nav_client = Nav2Client(self)

        self.map = Map(self)

        self._tf_subscriber = TFSubscriber(self, base_link_frame="base_link")

        self._object_detector = ObjectDetector(
            './yolo_models',
            confidence_threshold=0.3
        )
        self._target_selector = TargetSelector(
            persistence_frames=10,
            distance_threshold=100.00
        )

        self._target_reached_detector = TargetReachedDetector(
            self,
            target_reached_threshold=1.5 # meters
        )

        self._camera_processor = CameraProcessor(
            self._bridge,
            self._object_detector,
            self._target_selector
        )

        self._depth_camera_processor = DepthCameraProcessor(
            self._bridge,
            self._tf_subscriber.tf_buffer,
        )

        self._lidar_processor = LidarProcessor()

        self.root = self.create_behaviour_tree()

        # LLM agent integration
        self._llm_api = LLMAPI()
        self._llm_history: List[BaseMessage] = []
        self._llm_requests: "queue.Queue[Optional[str]]" = queue.Queue()

        self._tick_period = 0.1
        self._shutdown_event = threading.Event()
        self._tick_thread = threading.Thread(
            target=self._run_tree_loop,
            name="behaviour-tree-tick",
            daemon=True,
        )

        self._llm_thread = threading.Thread(
            target=self._run_llm_loop,
            name="llm-agent-loop",
            daemon=True,
        )

        self._tick_thread.start()
        self._llm_thread.start()


        self.create_subscription(
            Image,
            "/oakd/rgb/preview/image_raw",
            self._camera_processor.handle,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self._depth_camera_processor.set_camera_intrinsics,
            10
        )

        self.create_subscription(
            Image,
            "/oakd/rgb/preview/depth",
            self._depth_camera_processor.handle,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self._lidar_processor.handle,
            10
        )

        self.create_subscription(
            String,
            'llm_instruction',
            self._handle_llm_instruction,
            10,
        )

    def create_behaviour_tree(self):
        root = py_trees.composites.Selector("Root", memory=False)
        obstacle_sequence = py_trees.composites.Sequence("HandleObstacle", memory=False)

        check_lidar = CheckLidar("CheckLidarObstacle")
        check_lidar.setup()
        obstacle_detected = py_trees.decorators.Inverter(name="ObstacleDetected", child=check_lidar)

        navigation_goal_idle = NavigationGoalIdle("NavigationGoalIdle")

        turn_around = TurnAround("TurnAround")
        turn_around.setup(self._twist, self._cmd_publisher)

        user_command_executor = UserCommandExecutor("UserCommandExecutor", self._nav_client, self)
        user_command_executor.setup(self._twist, self._cmd_publisher)

        obstacle_sequence.add_children([obstacle_detected, navigation_goal_idle, turn_around])
        root.add_children([obstacle_sequence, user_command_executor])
        return root

    def tick(self):
        self.root.tick_once()
        return

    # LLM agent handling

    def submit_llm_instruction(self, instruction: str) -> None:
        """Queue a natural language instruction for the LLM controller."""
        if not instruction:
            return
        self._llm_requests.put(instruction)

    def _handle_llm_instruction(self, msg: String) -> None:
        instruction = msg.data.strip()
        if not instruction:
            return
        self.get_logger().info(f"Received LLM instruction: {instruction}")
        self.submit_llm_instruction(instruction)

    def _run_llm_loop(self) -> None:
        while not self._shutdown_event.is_set():
            try:
                instruction = self._llm_requests.get(timeout=0.5)
            except queue.Empty:
                continue

            if instruction is None or self._shutdown_event.is_set():
                break

            try:
                result = self._llm_api.run(instruction, history=self._llm_history)
                self._llm_history = list(result.get("chat_history", []))
                response_text = result.get("output", "")
                if response_text:
                    self.get_logger().info(f"LLM response: {response_text}")
            except Exception as exc:  # noqa: BLE001 - capture LLM issues without crashing node
                self.get_logger().error(f"LLM processing failed: {exc}")

    def _run_tree_loop(self) -> None:
        while not self._shutdown_event.is_set():
            if self._blackboard.is_behaviour_tree_paused():
                # Sleep in short intervals so resume responds quickly.
                self._shutdown_event.wait(timeout=self._tick_period)
                continue

            start = time.perf_counter()
            self.tick()

            elapsed = time.perf_counter() - start
            sleep_time = max(0.0, self._tick_period - elapsed)
            if sleep_time:
                # Wait with timeout so we exit promptly when shutting down
                self._shutdown_event.wait(timeout=sleep_time)

    def destroy_node(self):
        self._shutdown_event.set()
        self._llm_requests.put(None)
        if self._tick_thread.is_alive():
            self._tick_thread.join(timeout=1.0)
        if self._llm_thread.is_alive():
            self._llm_thread.join(timeout=1.0)
        return super().destroy_node()
