#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from web.backend.mission_board_server import MissionBoardServer
from shared.blackboard.blackboard import Blackboard

import base64
import pickle


class BlackboardSubscriber(Node):
    def __init__(self, namespace):
        super().__init__('blackboard_subscriber', namespace=namespace)

        self._mission_board_server = MissionBoardServer(
            instruction_handler=self.submit_llm_instruction
        )
        self._mission_board_server.start()

        self._blackboard = Blackboard(disable_event_bus_subscription=True)

        self.subscription = self.create_subscription(
            String,
            f'{namespace}/blackboard',
            self.listener_callback,
            10
        )

        self.get_logger().info(
            f"Blackboard subscriber started; listening on {namespace}/blackboard"
        )

        self._llm_instruction_publisher = self.create_publisher(
            String,
            f'{namespace}/llm_instruction',
            10
        )

    def listener_callback(self, msg: String):
        try:
            decoded_bytes = base64.b64decode(msg.data)
            dictionary = pickle.loads(decoded_bytes)
            self._blackboard.load_dict(dictionary)

        except Exception as e:
            self.get_logger().error(f"Failed to decode/unpickle data: {e}")

    def submit_llm_instruction(self, instruction):
        msg = String()
        msg.data = instruction
        self._llm_instruction_publisher.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Blackboard Subscriber Node')
    parser.add_argument(
        '--namespace',
        type=str,
        default='/robot_1',
        help='Robot namespace (e.g. /robot_1, /robot_2)'
    )

    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = BlackboardSubscriber(namespace=parsed_args.namespace)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
    main()
