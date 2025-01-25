import subsystem
import sensors
import wpilib
import config
import constants


class Robot:
    drivetrain = subsystem.Drivetrain()


class Pneumatics:
    pass


class Sensors:
    right_cam = sensors.PhotonCamCustom(config.right_cam_name, constants.robot_to_right_cam)
    left_cam = sensors.PhotonCamCustom(config.left_cam_name, constants.robot_to_left_cam)
    cam_controller = sensors.PhotonController([left_cam, right_cam])


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    odometry = sensors.FieldOdometry(Robot.drivetrain, Sensors.cam_controller, constants.field_width, constants.field_length)
