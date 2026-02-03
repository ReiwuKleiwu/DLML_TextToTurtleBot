import argparse
import rclpy
from core.nodes.text_to_turtlebot_node import TextToTurtlebotNode
from dotenv import load_dotenv

def main(args=None):
    load_dotenv()

    parser = argparse.ArgumentParser(description='Text to Turtlebot Node')
    parser.add_argument(
        '--namespace',
        type=str,
        default='/robot_1',
        help='Robot namespace (e.g. /robot_1, /robot_2)'
    )
    parser.add_argument(
        '--use_turtlebot_sim',
        action='store_true',
        default=False,
        help='Use Turtlebot simulation'
    )

    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    node = TextToTurtlebotNode(
        namespace=parsed_args.namespace,
        use_turtlebot_sim=parsed_args.use_turtlebot_sim
    )

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
