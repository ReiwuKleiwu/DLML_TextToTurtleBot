import threading
import time

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
from behaviours.drive_forward import DriveForward
from behaviours.turn_around import TurnAround
from behaviours.check_lidar import CheckLidar
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from events.event_bus import EventBus
from blackboard.blackboard import Blackboard
from map.map import Map

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

        self._tick_period = 0.1
        self._shutdown_event = threading.Event()
        self._tick_thread = threading.Thread(
            target=self._run_tree_loop,
            name="behaviour-tree-tick",
            daemon=True,
        )
        
        self._tick_thread.start()


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

    def create_behaviour_tree(self):
        root = py_trees.composites.Selector("Root", memory=False)
        sequence = py_trees.composites.Sequence("Sequence", memory=False)
        
        check_lidar = CheckLidar("CheckLidarObstacle")
        check_lidar.setup()

        turn_around = TurnAround("TurnAround")
        turn_around.setup(self._twist, self._cmd_publisher)

        drive_forward = DriveForward("DriveForward")
        drive_forward.setup(self._twist, self._cmd_publisher)

        sequence.add_children([check_lidar, turn_around])
        root.add_children([sequence, drive_forward])
        return root

    def tick(self):
        # self.root.tick_once()
        return

    def _run_tree_loop(self) -> None:
        while not self._shutdown_event.is_set():
            start = time.perf_counter()
            self.tick()

            elapsed = time.perf_counter() - start
            sleep_time = max(0.0, self._tick_period - elapsed)
            if sleep_time:
                # Wait with timeout so we exit promptly when shutting down
                self._shutdown_event.wait(timeout=sleep_time)

    def destroy_node(self):
        self._shutdown_event.set()
        if self._tick_thread.is_alive():
            self._tick_thread.join(timeout=1.0)
        return super().destroy_node()
