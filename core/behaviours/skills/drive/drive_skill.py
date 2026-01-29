import py_trees

from core.behaviours.skills.drive.drive_motion import DriveMotion
from core.behaviours.skills.drive.prepare_drive_goal import PrepareDriveGoal
from core.commands.user_command import UserCommand
from shared.utils.twist_wrapper import TwistWrapper

class DriveSkill(py_trees.composites.Sequence):
    """Sequential skill that carries out a simple distance-based drive command."""

    def __init__(self, name: str, command: UserCommand, speed: float = 0.25, tolerance: float = 0.0) -> None:
        super().__init__(name, memory=True)
        self.command = command
        self.prepare = PrepareDriveGoal("PrepareDriveGoal", command)
        self.drive = DriveMotion("DriveMotion", command, speed=speed, tolerance=tolerance)
        self.add_children([self.prepare, self.drive])

    def setup(self, twist: TwistWrapper, publisher) -> None:  # type: ignore[override]
        self.drive.setup(twist, publisher)
