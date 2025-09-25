import py_trees

from behaviours.skills.rotate.prepare_rotate_goal import PrepareRotateGoal
from behaviours.skills.rotate.rotate_motion import RotateMotion
from commands.user_command import UserCommand
from utils.twist_wrapper import TwistWrapper


class RotateSkill(py_trees.composites.Sequence):
    """Sequential skill that carries out a rotation command."""

    def __init__(self, name: str, command: UserCommand, angular_speed: float = 0.8, tolerance_deg: float = 0.0) -> None:
        super().__init__(name, memory=True)
        self.command = command
        self.prepare = PrepareRotateGoal("PrepareRotateGoal", command)
        self.rotate = RotateMotion("RotateMotion", command, angular_speed=angular_speed, tolerance_deg=tolerance_deg)
        self.add_children([self.prepare, self.rotate])

    def setup(self, twist: TwistWrapper, publisher) -> None:  # type: ignore[override]
        self.rotate.setup(twist, publisher)
