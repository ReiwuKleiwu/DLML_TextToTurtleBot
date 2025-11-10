import py_trees
from py_trees.common import Status

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from perception.detection.object_detector import DetectedObject


class TargetSelected(py_trees.behaviour.Behaviour):
    """Checks whether the desired object is currently selected by perception."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = Blackboard()

    def update(self) -> Status:
        selected_target: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)

        if not selected_target:
            return Status.FAILURE

        return Status.SUCCESS
