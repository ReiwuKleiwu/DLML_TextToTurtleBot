#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OakDListenerNode(Node):
    """Node that listens to Oak-D camera topics and prints the messages."""
    
    def __init__(self):
        super().__init__('oakd_listener_node')
        
        # Definiere das Topic, das du abonnieren möchtest
        self.declare_parameter('oakd_topic', '/robot_1/oakd/rgb/preview/image_raw')
        
        topic_name = str(self.get_parameter('oakd_topic').value)

        # Definiere QoS-Einstellungen für höhere Leistung und weniger Delay
        qos_profile = QoSProfile(
            depth=100,  # Setze die Queue-Tiefe höher
            reliability=ReliabilityPolicy.RELIABLE,  # Setze Zuverlässigkeit auf RELIABLE
            history=HistoryPolicy.KEEP_LAST,  # Behalte nur die letzten Nachrichten
        )

        # Erstelle die Subscription mit den optimierten QoS-Einstellungen
        self.subscription = self.create_subscription(
            Image, 
            topic_name, 
            self.listener_callback, 
            qos_profile
        )

        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def listener_callback(self, msg: Image):
        """Callback-Funktion, die bei jedem Eintreffen einer Nachricht ausgeführt wird."""
        self.get_logger().info(f"Received Image message: Header - {msg.header}")

def main(args=None):
    rclpy.init(args=args)
    node = OakDListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
