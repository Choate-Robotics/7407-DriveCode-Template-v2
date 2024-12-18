import math
import time

import ntcore, config

from toolkit.command import SubsystemCommand
from subsystem.drivetrain import Drivetrain
from toolkit.utils.units import radians
from toolkit.utils.toolkit_math import bounded_angle_diff, rotate_vector

from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from robot_systems import Sensors, Field
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from command.autonomous.trajectory import Choreo_Trajectory
from enum import Enum
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from robot import Robot

from robot_systems import Field

class AngleType(Enum):

    path = 0
    calculate = 1

class FollowChoreoPath(SubsystemCommand[Drivetrain]):
    """
    Command to follow a choreo path
    """
    def __init__(self,
                 subsystem: Drivetrain,
                 choreo_path: Choreo_Trajectory,
                 period: float = config.period,
                 theta_f: float = AngleType.path
                 ):
        super().__init__(subsystem)
        self.choreo_path = choreo_path
        self.period = period
        self.theta_f = theta_f
        AutoBuilder.configureHolonomic(
            self.subsystem.odometry.getPose(), # Robot pose supplier
            self.subsystem.reset_odometry_auto(), # Method to reset odometry (will be called if your auto has a starting pose)
            self.subsystem.chassis_speeds.fromRobotRelativeSpeeds(), # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            # implement this function using setrobotcentric overload/make new mtd
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def initialize(self):
        pass

    def execute(self):
        pass

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool):
        pass

