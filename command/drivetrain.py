from __future__ import annotations

import wpilib
import ntcore

from robot_systems import Field
from toolkit.command import SubsystemCommand

import config
from subsystem import Drivetrain
from toolkit.utils.toolkit_math import bounded_angle_diff
from math import radians
from wpimath.units import seconds
from enum import Enum
import logging
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians

def deadzone(x, d=config.drivetrain_deadzone):
    if abs(x) < d:
        return 0
    if x < 0:
        return (x+d)/(1-d)
    return (x-d)/(1-d)

def curve(x, d=config.drivetrain_deadzone, c=config.drivetrain_curve):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1*math.pow((-1*(x+d)/(1-d)), c)
    return math.pow(((x-d)/(1-d)), c)

def bound_angle(degrees: float):
    degrees = degrees % 360
    if degrees > 180:
        degrees -= 360
    return degrees


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    """
    Main drive command
    """
    driver_centric = False
    driver_centric_reversed = True

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * -1,
            self.subsystem.axis_dy.value * 1,
            self.subsystem.axis_rotation.value,
        )

        dx = deadzone(dx)
        dy = deadzone(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        
        d_theta *= self.subsystem.max_angular_vel
        

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), -d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx, d_theta))

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False

class DrivetrainZero(SubsystemCommand[Drivetrain]):
    """
    Zeroes drivetrain
    """

    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        self.subsystem.reset_gyro(config.drivetrain_zero)
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        logging.info("Successfully re-zeroed swerve pods.")
        ...

class DrivetrainXMode(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.x_mode()
    
    def execute(self) -> None:
        pass

    def isFinished(self):
        return False
    
    def end(self, interrupted: bool) -> None:
        pass

class DriveToPose(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, pose: Pose2d = None):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.pose = pose
        self.current_pose: Pose2d

        self.x_controller = PIDController(4, 0, 0, config.period)
        self.y_controller = PIDController(4, 0, 0, config.period)
        self.theta_controller = PIDController(5.5, 0, 0, config.period)

        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("drive to pose")


    def initialize(self):
        if self.pose == None:
            self.pose = self.subsystem.get_pose()
        
        self.theta_controller.enableContinuousInput(0, math.radians(360))

        self.x_controller.setTolerance(0.05)
        self.y_controller.setTolerance(0.05)
        self.theta_controller.setTolerance(math.radians(1))

        self.x_controller.setSetpoint(self.pose.X())
        self.y_controller.setSetpoint(self.pose.Y())
        self.theta_controller.setSetpoint(self.pose.rotation().radians())

    def execute(self):
        self.current_pose = self.subsystem.get_pose()

        vx = self.x_controller.calculate(self.current_pose.X())
        vy = self.y_controller.calculate(self.current_pose.Y())
        vtheta = self.theta_controller.calculate(self.current_pose.rotation().radians())

        self.subsystem.set_driver_centric((vx, vy), vtheta)

        self.nt.putNumber("goal x", self.pose.X())
        self.nt.putNumber("goal y", self.pose.Y())
        self.nt.putNumber("goal theta", self.pose.rotation().degrees())

        self.nt.putNumber("current x", self.current_pose.X())
        self.nt.putNumber("current y", self.current_pose.Y())
        self.nt.putNumber("current theta", math.degrees(bounded_angle_diff(self.current_pose.rotation().radians(), 0)))

    def isFinished(self) -> bool:
        return self.x_controller.atSetpoint() and self.y_controller.atSetpoint() and self.theta_controller.atSetpoint()
    
    def end(self, interrupted):
        self.subsystem.set_driver_centric((0, 0), 0)