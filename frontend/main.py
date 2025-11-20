#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from frontend.web.mission_board_server import MissionBoardServer
from shared.blackboard.blackboard import Blackboard

import base64
import pickle


class BlackboardSubscriber(Node):
    def __init__(self):
        super().__init__('blackboard_subscriber')
        self._mission_board_server = MissionBoardServer(instruction_handler=self.submit_llm_instruction)
        self._mission_board_server.start()

        self._blackboard = Blackboard(disable_event_bus_subscription=True)

        self.subscription = self.create_subscription(
            String,
            '/blackboard',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Blackboard subscriber started; listening on /blackboard")

        self._llm_instruction_publisher = self.create_publisher(String, 'llm_instruction', 10)

    def listener_callback(self, msg: String):
        data_b64 = msg.data

        try:
            # Base64 decode
            decoded_bytes = base64.b64decode(data_b64)

            # Unpickle
            dictionary = pickle.loads(decoded_bytes)
            self._blackboard.load_dict(dictionary)

            #self.get_logger().info(f"Received object: {dictionary.keys()}")

        except Exception as e:
            self.get_logger().error(f"Failed to decode/unpickle data: {e}")
    
    def submit_llm_instruction(self, instruction):
        msg = String()
        msg.data = instruction
        self._llm_instruction_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlackboardSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
