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
        object_found_data = self.state_machine.get_current_state().data.get("object_found_data", None)
        if object_found_data is None: 
            print(f"[TARGET NAVIGATION - ERROR]: Detected target object is not defined.")
            return
        
        bounding_box_width = object_found_data["bounding_box_coordinates"]["x2"] - object_found_data["bounding_box_coordinates"]["x1"]
        bounding_box_center = object_found_data["bounding_box_coordinates"]["x1"] + (bounding_box_width / 2)

        camera_resolution_width = object_found_data["camera"]["width"]
        image_center = camera_resolution_width / 2
        offset_from_image_center = bounding_box_center - image_center

        # print(f"[TARGET NAVIGATION]: Detected object bounding box offset from center: {offset_from_image_center}")

        if abs(offset_from_image_center) >= 25:
            normalized_turn_direction = self.map_to_minus1_to_1(offset_from_image_center, -(camera_resolution_width / 2), camera_resolution_width / 2)
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1 * normalized_turn_direction
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.cmd_publisher.publish(self.twist)

    def map_to_minus1_to_1(self, x, a, b):
        return 2 * (x - a) / (b - a) - 1
