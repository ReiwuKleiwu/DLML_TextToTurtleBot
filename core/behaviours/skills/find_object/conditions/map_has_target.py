from typing import List, Optional

import py_trees
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from core.map.map import PersistentTrackedObject
from core.perception.detection.object_detector import DetectedObject


class MapHasTarget(py_trees.behaviour.Behaviour):
    """Checks whether the requested object class exists in the persistent map."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = Blackboard()

    def update(self) -> Status:
        persistent_objects: List[PersistentTrackedObject] = self._blackboard.get(BlackboardDataKey.ROBOT_MAP) 
        target_obect_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)

        if not persistent_objects or not target_obect_class:
            return Status.FAILURE

        for object in persistent_objects:            
            if object.detected_object.name.lower() == target_obect_class:
                return Status.SUCCESS
            
        return Status.FAILURE
