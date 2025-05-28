from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Publisher


from classes.behaviors.target_navigation.TargetNavigationStrategy import TargetNavigationStrategy
from classes.controllers.StateMachine import StateMachine

class SimpleTargetNavigation(TargetNavigationStrategy): 
    def __init__(self, state_machine: StateMachine, twist: Twist | TwistStamped, cmd_publisher: Publisher):
        self.state_machine = state_machine
        self.twist = twist
        self.cmd_publisher = cmd_publisher

    def execute(self):
        detected_object = self.state_machine.get_current_state().data.get("detected_target_object", None)
        if detected_object is None: 
            print(f"[TARGET NAVIGATION - ERROR]: Detected target object is not defined.")
            return
        
        bounding_box_width = detected_object["bounding_box_coordinates"]["x2"] - detected_object["bounding_box_coordinates"]["x1"]
        bounding_box_center = detected_object["bounding_box_coordinates"]["x1"] + (bounding_box_width / 2)

        # Camera has a resolution of 250x250
        image_center = 150
        offset_from_image_center = bounding_box_center - image_center

        print(f"[TARGET NAVIGATION]: Detected object bounding box offset from center: {offset_from_image_center}")

        if abs(offset_from_image_center) >= 25:
            normalized_turn_direction = self.map_to_minus1_to_1(offset_from_image_center, -150, 150)
            self.twist.twist.linear.x = 0.0
            self.twist.twist.angular.z = -0.1 * normalized_turn_direction
        else:
            self.twist.twist.linear.x = 0.2
            self.twist.twist.angular.z = 0.0 

        self.cmd_publisher.publish(self.twist)

    def map_to_minus1_to_1(self, x, a, b):
        return 2 * (x - a) / (b - a) - 1
