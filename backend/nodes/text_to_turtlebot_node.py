import queue
import re
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from cv_bridge import CvBridge
from backend.perception.camera.camera_processor import CameraProcessor
from backend.perception.camera.depth_camera_processor import DepthCameraProcessor
from backend.perception.detection.object_detector import ObjectDetector
from backend.perception.detection.target_reached_detector import TargetReachedDetector
from backend.perception.detection.target_selector import TargetSelector
from backend.perception.tf.tf_subscriber import TFSubscriber
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import Twist, TwistStamped
from shared.utils.twist_wrapper import TwistWrapper
from backend.behaviours.actions.turn_around import TurnAround
from backend.behaviours.conditions.check_lidar import CheckLidar
from backend.behaviours.conditions.navigation_goal_idle import NavigationGoalIdle
from backend.behaviours.user_command_executor import UserCommandExecutor
from backend.navigation.nav2_client import Nav2Client
from backend.navigation.docking_client import DockingClient
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import String
from shared.events.event_bus import EventBus
from shared.blackboard.blackboard import Blackboard
from backend.map.map import Map
from backend.natural_language_processing.llm_api import LLMAPI
from backend.natural_language_processing.text_to_speech import (
    TextToSpeechError,
    TextToSpeechService,
)
from langchain_core.messages import BaseMessage

from backend.perception.lidar.lidar_processor import LidarProcessor
from backend.perception.lidar.lidar_object_coordinate_processor import LidarObjectCoordinateProcessor

from std_msgs.msg import String

class TextToTurtlebotNode(Node):
    def __init__(self, namespace: str = '', use_turtlebot_sim: bool = False) -> None:
        super().__init__('TextToTurtlebotNode', namespace=namespace)
        self.get_logger().info('Text to Turtlebot Node has been started.')

        self._event_bus = EventBus()
        self._blackboard = Blackboard()

        self._bridge = CvBridge()

        self._twist = TwistWrapper(use_stamped=use_turtlebot_sim)
        msg_type = TwistStamped if use_turtlebot_sim else Twist
        self._cmd_publisher = self.create_publisher(msg_type, f'{namespace}/cmd_vel', 10)
        self._blackboard_publisher = self.create_publisher(String, f'{namespace}/blackboard', 10)

        self._nav_client = Nav2Client(self)
        self._docking_client = DockingClient(self)

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

        coordinate_mode = "lidar"  # set to "lidar" to use LIDAR-based object coordinates
        if coordinate_mode not in {"depth", "lidar"}:
            coordinate_mode = "depth"
        self._coordinate_mode = coordinate_mode
        self.get_logger().info(f"Object coordinate mode: {self._coordinate_mode}")

        self._depth_camera_processor: Optional[DepthCameraProcessor] = None
        self._lidar_coordinate_processor: Optional[LidarObjectCoordinateProcessor] = None

        if self._coordinate_mode == "depth":
            self._depth_camera_processor = DepthCameraProcessor(
                self._bridge,
                self._tf_subscriber.tf_buffer,
            )
        else:
            self._lidar_coordinate_processor = LidarObjectCoordinateProcessor(
                self._tf_subscriber.tf_buffer,
            )

        self._lidar_processor = LidarProcessor()

        self.root = self.create_behaviour_tree()

        # LLM agent integration
        self._llm_api = LLMAPI()
        self._tts_service = TextToSpeechService(self._llm_api.get_tts_settings())
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

        self._blackboard_thread = threading.Thread(
            target=self._run_blackboard_loop,
            name="blackboard-loop",
            daemon=True,
        )

        self._tick_thread.start()
        self._llm_thread.start()
        self._blackboard_thread.start()


        self.create_subscription(
            Image,
            f'{namespace}oakd/rgb/preview/image_raw',
            self._camera_processor.handle,
            10
        )

        self.create_subscription(
            CameraInfo,
             f'{namespace}/oakd/rgb/preview/camera_info',
            self._handle_camera_info,
            10
        )

        if self._depth_camera_processor is not None:
            self.create_subscription(
                Image,
                f'{namespace}/oakd/stereo/image_raw' if not use_turtlebot_sim
                else f'{namespace}/oakd/rgb/preview/depth',
                self._depth_camera_processor.handle,
                10
            )

        self.create_subscription(
            LaserScan,
             f'{namespace}/scan',
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

        user_command_executor = UserCommandExecutor(
            "UserCommandExecutor",
            self._nav_client,
            self._docking_client,
            self,
        )
        user_command_executor.setup(self._twist, self._cmd_publisher)

        obstacle_sequence.add_children([obstacle_detected, navigation_goal_idle, turn_around])
        root.add_children([obstacle_sequence, user_command_executor])
        return root

    def tick(self):
        self.root.tick_once()
        return

    # LLM agent handling

    def _handle_camera_info(self, msg: CameraInfo) -> None:
        if self._depth_camera_processor is not None:
            self._depth_camera_processor.set_camera_intrinsics(msg)
        if self._lidar_coordinate_processor is not None:
            self._lidar_coordinate_processor.set_camera_intrinsics(msg)

    def submit_llm_instruction(self, instruction: str) -> None:
        """Queue a natural language instruction for the LLM controller."""
        if not instruction:
            return
        cleaned = instruction.strip()
        if not cleaned:
            return
        self._blackboard.append_chat_message("user", cleaned)
        self._llm_requests.put(cleaned)

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
                    metadata = self._maybe_generate_tts_metadata(response_text)
                    self._blackboard.append_chat_message(
                        "assistant",
                        response_text,
                        metadata=metadata,
                    )
            except Exception as exc:  # noqa: BLE001 - capture LLM issues without crashing node
                self.get_logger().error(f"LLM processing failed: {exc}")
                self._blackboard.append_chat_message(
                    "system",
                    f"LLM processing failed: {exc}",
                    metadata={"error": True},
                )

    def _run_blackboard_loop(self) -> None:
        while not self._shutdown_event.is_set():
            self._blackboard.publish(self._blackboard_publisher)
            time.sleep(0.25)

    def _maybe_generate_tts_metadata(self, response_text: str) -> Optional[Dict[str, Any]]:
        """Convert the assistant response to speech when enabled."""
        service = getattr(self, "_tts_service", None)
        if service is None or not service.is_enabled():
            return None

        tts_text = self._extract_tts_text(response_text)
        if not tts_text:
            self.get_logger().debug("No TTS_Text segment found; skipping audio generation.")
            return None

        try:
            audio_path = service.synthesise(tts_text)
        except TextToSpeechError as exc:
            self.get_logger().warning(f"TTS synthesis failed: {exc}")
            return None

        if not audio_path:
            return None

        audio_url = self._resolve_audio_url(audio_path)
        if not audio_url:
            self.get_logger().warning(f"Unable to resolve audio URL for {audio_path}")
            return None

        audio_format = audio_path.suffix.lstrip('.') or service.settings.audio_format

        return {
            "audio": {
                "url": audio_url,
                "format": audio_format,
                "provider": service.settings.provider,
                "text": tts_text,
            }
        }

    def _resolve_audio_url(self, audio_path: Path) -> Optional[str]:
        service = getattr(self, "_tts_service", None)
        if service is None:
            return None

        try:
            base_dir = service.settings.resolved_output_dir().resolve()
            relative_path = audio_path.resolve().relative_to(base_dir)
        except (ValueError, OSError):
            return None

        return f"/audio/{relative_path.as_posix()}"

    @staticmethod
    def _extract_tts_text(response_text: str) -> Optional[str]:
        if not response_text:
            return None

        # Try quoted form first: TTS_Text: "..."
        quoted_match = re.search(r"TTS_Text\s*:\s*\"(?P<text>.*?)\"", response_text, re.DOTALL)
        if quoted_match:
            text = quoted_match.group("text").strip()
            return text if text else None

        # Fallback: take text after TTS_Text: up to newline
        fallback_match = re.search(r"TTS_Text\s*:\s*(?P<text>.+)", response_text)
        if fallback_match:
            text = fallback_match.group("text").strip()
            if text:
                # Stop at first explicit newline to avoid capturing following sections
                first_line = text.splitlines()[0].strip().strip('"')
                return first_line if first_line else None
        return None

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
        if hasattr(self, "_mission_board_server"):
            self._mission_board_server.stop()
        return super().destroy_node()
