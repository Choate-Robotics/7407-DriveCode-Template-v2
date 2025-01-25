from utils import LocalLogger
from oi.keymap import Keymap
import command
import constants
import math
from robot_systems import Robot
from commands2 import InstantCommand
from wpimath.geometry import Pose2d

log = LocalLogger("OI")


class OI:
    @staticmethod
    def init() -> None:
        log.info("Initializing OI...")

    @staticmethod
    def map_controls():
        log.info("Mapping controls...")
        pass

        Keymap.Drivetrain.RESET_GYRO.onTrue(
            command.DrivetrainZero(Robot.drivetrain)) \
            .onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(
            command.DrivetrainXMode(Robot.drivetrain)
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.DRIVE_TO_RIGHT_POSE.onTrue(
            command.DriveToPose(Robot.drivetrain, Pose2d(14.4, 4.15, math.radians(180)))
        ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))

        Keymap.Drivetrain.RESET_POSE.onTrue(
            InstantCommand(lambda: Robot.drivetrain.reset_odometry(Pose2d(constants.field_length/2, constants.field_width/2, math.radians(180))))
        )