from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from toolkit.command import BasicCommand

class DriveChoreoPath(BasicCommand):
    """
    Zeroes drivetrain
    """

    def __init__(self, c_path_name):
        super().__init__()
        self.c_path = c_path_name
        self.

    def initialize(self) -> None:
        pass
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass