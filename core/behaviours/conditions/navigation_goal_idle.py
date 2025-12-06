from py_trees.behaviour import Behaviour
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey


class NavigationGoalIdle(Behaviour):
    """Succeeds when no navigation goal is currently active."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = Blackboard()
        self._active_statuses = {"sent", "accepted"}

    def update(self) -> Status:
        status = self._blackboard.get(BlackboardDataKey.NAVIGATION_STATUS)
        if status in self._active_statuses:
            return Status.FAILURE
        return Status.SUCCESS
