#!/usr/bin/env python3
"""ROS 2 node that republishes a camera image topic with a configurable delay."""
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


class ImageDelayNode(Node):
    """Relay image frames with a fixed latency to simulate network lag."""

    def __init__(self) -> None:
        super().__init__('image_delay_node')

        # Declare configurable parameters
        self.declare_parameter('input_topic', '/robot_1/oakd/rgb/preview/image_raw')
        self.declare_parameter('output_topic', '/robot_1/oakd/rgb/preview/image_raw_delayed')
        self.declare_parameter('delay_seconds', 0.01)
        self.declare_parameter('queue_limit', 100)

        # Cache parameter values
        self.delay = float(self.get_parameter('delay_seconds').value)
        self.queue_limit = int(self.get_parameter('queue_limit').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        qos = QoSProfile(depth=10)
        self.buffer: deque[tuple[float, Image]] = deque()

        self.subscription = self.create_subscription(
            Image, input_topic, self._on_image, qos
        )
        self.publisher = self.create_publisher(Image, output_topic, qos)

        # Timer checks the queue frequently for frames ready to send downstream
        self.timer = self.create_timer(0.01, self._publish_ready_frames)

        # self.get_logger().info(
        #     'Delaying images from %s -> %s by %.3f seconds (queue_limit=%d)',
        #     input_topic,
        #     output_topic,
        #     self.delay,
        #     self.queue_limit,
        # )

    def _on_image(self, msg: Image) -> None:
        """Store the latest frame and trim the queue if it grows too large."""
        if len(self.buffer) >= self.queue_limit:
            dropped_time, _ = self.buffer.popleft()
            self.get_logger().debug(
                'Dropping frame recorded at %.6f due to queue limit', dropped_time
            )
        self.buffer.append((time.monotonic(), msg))

    def _publish_ready_frames(self) -> None:
        """Publish frames once they have aged past the configured delay."""
        now = time.monotonic()
        while self.buffer and now - self.buffer[0][0] >= self.delay:
            _, msg = self.buffer.popleft()
            # Uncomment to restamp with publish time instead of original acquisition time
            # msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageDelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
