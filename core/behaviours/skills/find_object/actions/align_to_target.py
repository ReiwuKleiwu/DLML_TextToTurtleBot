import py_trees
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from core.perception.detection.object_detector import DetectedObject
from shared.utils.twist_wrapper import TwistWrapper


class AlignToTarget(py_trees.behaviour.Behaviour):
    """Rotates the robot so the selected object is centered in the camera view."""

    def __init__(self, name: str, twist: TwistWrapper, publisher) -> None:
        super().__init__(name)
        self._twist = twist
        self._publisher = publisher
        self._blackboard = Blackboard()
        self._camera_resolution = self._blackboard.get(BlackboardDataKey.CAMERA_RESOLUTION)
        self._tolerance = 20.0

    def update(self) -> Status:
        self._camera_resolution = self._blackboard.get(BlackboardDataKey.CAMERA_RESOLUTION)
        if not self._camera_resolution:
            return Status.RUNNING
        
        image_center = self._camera_resolution['width'] / 2

        selected_object: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)
        if not selected_object:
            self._stop()
            return Status.FAILURE
        
        selected_object_bb_width = selected_object.x2 - selected_object.x1
        selected_object_bb_center = selected_object.x1 + (selected_object_bb_width / 2)

        error = selected_object_bb_center - image_center

        if abs(error) <= self._tolerance:
            self._stop()
            return Status.SUCCESS

        normalized_turn_direction = self._map_to_minus1_to_1(error, -self._camera_resolution['width']/2, self._camera_resolution['width']/2)
        self._twist.reset()
        self._twist.angular.z = -1 * normalized_turn_direction
        self._publisher.publish(self._twist.get_message())
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._stop()

    def _map_to_minus1_to_1(self, x, a, b):
        return 2 * (x - a) / (b - a) - 1

    def _stop(self) -> None:
        self._twist.reset()
        self._publisher.publish(self._twist.get_message())
