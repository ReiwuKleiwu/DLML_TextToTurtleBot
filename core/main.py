import rclpy
from core.nodes.text_to_turtlebot_node import TextToTurtlebotNode

def main(args=None):
    rclpy.init(args=args)
    node = TextToTurtlebotNode(namespace='/robot_2', use_turtlebot_sim=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()