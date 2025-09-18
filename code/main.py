import rclpy

from classes.nodes.TextToTurtlebotNode import TextToTurtlebotNode
from classes.controllers.MovementThread import MovementThread


def main(args=None):
    rclpy.init(args=args)

    text_to_turtlebot_node = TextToTurtlebotNode(namespace="", use_turtlebot_sim=True)

    text_to_turtlebot_node.find_target('chair')
    text_to_turtlebot_node.find_target('person')
    text_to_turtlebot_node.find_target('chair')


    try:
        movement_thread = MovementThread(2, text_to_turtlebot_node)
        rclpy.spin(text_to_turtlebot_node)
        text_to_turtlebot_node.destroy_node()
        rclpy.shutdown()
    finally:
        movement_thread.end()
    

if __name__ == '__main__':
    main()
