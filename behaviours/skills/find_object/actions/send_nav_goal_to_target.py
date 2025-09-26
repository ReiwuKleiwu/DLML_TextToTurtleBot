import math
from typing import List, Optional

import py_trees
from py_trees.common import Status
from geometry_msgs.msg import PoseStamped, Quaternion

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from map.map import PersistentTrackedObject
from navigation.nav2_client import Nav2Client
from perception.detection.object_detector import DetectedObject
from rclpy.node import Node


class SendNavGoalToTarget(py_trees.behaviour.Behaviour):
    """Sends a Nav2 goal to the first matching object stored in the map."""

    def __init__(self, name: str, node: Node, nav_client: Nav2Client) -> None:
        super().__init__(name)
        self._node = node
        self._nav_client = nav_client
        self._blackboard = Blackboard()
        self._goal_sent = False

    def update(self) -> Status:
        if self._goal_sent:
            return Status.SUCCESS

        target_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
        persistent_objects: List[PersistentTrackedObject] = self._blackboard.get(BlackboardDataKey.ROBOT_MAP)
        selected_target_object: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)

        target: Optional[DetectedObject] = None

        # Try to use the selected target object if it exists in the map
        if selected_target_object:
            target_in_map = self._find_target_in_robot_map(persistent_objects, selected_target_object)
            if target_in_map:
                target = target_in_map

        # Otherwise, find the first instance of the target class in the map
        if target is None:
            for object in persistent_objects:
                if object.detected_object.name.lower() == target_class.lower():
                    target = object
                    break

        if target is None:
            return Status.FAILURE
        
        if isinstance(target, PersistentTrackedObject):
            target = target.detected_object

        goal = PoseStamped()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(target.world_x or 0.0)
        goal.pose.position.y = float(target.world_y or 0.0)
        goal.pose.orientation = self._yaw_to_quaternion(0.0)

        if self._nav_client.send_goal(goal):
            self._goal_pose = goal
            self._goal_sent = True
            return Status.SUCCESS

        self.logger.warn("Failed to send Nav2 goal for target object")
        return Status.FAILURE
    
    def _find_target_in_robot_map(self, map: List[PersistentTrackedObject], target_object: DetectedObject) -> Optional[DetectedObject]:
        for object in map:
            if object.detected_object == target_object:
                return object
        return None
        

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half_yaw = yaw * 0.5
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        return quat
