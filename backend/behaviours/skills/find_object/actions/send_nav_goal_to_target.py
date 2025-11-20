import math
from typing import List, Optional

import py_trees
from py_trees.common import Status
from geometry_msgs.msg import PoseStamped, Quaternion

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from backend.map.map import PersistentTrackedObject
from backend.navigation.nav2_client import Nav2Client
from backend.perception.detection.object_detector import DetectedObject
from rclpy.node import Node


class SendNavGoalToTarget(py_trees.behaviour.Behaviour):
    """Sends a Nav2 goal to the first matching object stored in the map."""

    def __init__(self, name: str, node: Node, nav_client: Nav2Client) -> None:
        super().__init__(name)
        self._node = node
        self._nav_client = nav_client
        self._blackboard = Blackboard()
        self._goal_sent = False
        self._goal_pose: Optional[PoseStamped] = None
        self._approach_offset = 1.0  # meters

    def update(self) -> Status:
        if self._goal_sent:
            return Status.SUCCESS

        target_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
        persistent_objects: List[PersistentTrackedObject] = self._blackboard.get(BlackboardDataKey.ROBOT_MAP)
        selected_target_object: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)

        target: Optional[DetectedObject] = None

        # Try to use the selected target object if it exists in the map
        if selected_target_object:
            target_in_map = self._find_selected_target_in_robot_map(persistent_objects, selected_target_object)
            if target_in_map:
                target = target_in_map

        # Otherwise, find the first instance of the target class in the map
        if target is None:
            target = self._find_closest_target_in_robot_map(persistent_objects, target_class)

        if target is None:
            return Status.FAILURE
        
        if isinstance(target, PersistentTrackedObject):
            target = target.detected_object

        if target is None:
            return Status.FAILURE

        target_x = float(target.world_x or 0.0)
        target_y = float(target.world_y or 0.0)

        goal_x = target_x
        goal_y = target_y
        yaw = 0.0

        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)

        if robot_position is None:
            return Status.FAILURE

        robot_x = float(getattr(robot_position, "x", 0.0))
        robot_y = float(getattr(robot_position, "y", 0.0))
        to_target_x = target_x - robot_x
        to_target_y = target_y - robot_y
        distance = math.hypot(to_target_x, to_target_y)

        # Make sure Robot faces the Nav2 goal and stops at a distance offset
        if distance > 1e-3:
            if distance > self._approach_offset:
                scale = (distance - self._approach_offset) / distance
                goal_x = robot_x + to_target_x * scale
                goal_y = robot_y + to_target_y * scale
            else:
                goal_x = robot_x
                goal_y = robot_y

            yaw = math.atan2(target_y - goal_y, target_x - goal_x)

        goal = PoseStamped()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation = self._yaw_to_quaternion(yaw)

        if self._nav_client.send_goal(goal):
            self._goal_pose = goal
            self._goal_sent = True
            return Status.SUCCESS

        self.logger.error("Failed to send Nav2 goal for target object")
        return Status.FAILURE
    
    def _find_selected_target_in_robot_map(self, map: List[PersistentTrackedObject], target_object: DetectedObject) -> Optional[DetectedObject]:
        for object in map:
            if object.detected_object == target_object:
                return object
        return None

    def _find_closest_target_in_robot_map(self, map: List[PersistentTrackedObject], target_class: str) -> Optional[PersistentTrackedObject]:
        closest_object = None
        closest_distance = float('inf')

        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)

        for object in map:
            if object.detected_object.name.lower() != target_class.lower():
                continue

            distance = object.detected_object.distance_to_world_coordinates(robot_position.x, robot_position.y, robot_position.z)
            if distance < closest_distance:
                closest_distance = distance
                closest_object = object

        return closest_object if closest_object else None
        

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half_yaw = yaw * 0.5
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        return quat
