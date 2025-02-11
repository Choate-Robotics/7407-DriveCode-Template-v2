import subsystem
import sensors
import wpilib #noqa
import config
import constants
from utils.field import (
    FieldConstants,
    ReefFace,
    ReefHeight,
    Branch,
    Reef,
    Barge,
    StagingPositions,
    CoralStation,
    Processor,
    flip_poses,
    update_table,
    NT_Updater,
)


class Robot:
    drivetrain = subsystem.Drivetrain()
    led = subsystem.AddressableLEDStrip(
        config.leds_id,
        config.leds_size,
        config.leds_speed,
        config.leds_brightness,
        config.leds_saturation,
        config.leds_spacing,
        )


class Pneumatics:
    pass


class Sensors:
    right_cam = sensors.PhotonCamCustom(config.right_cam_name, constants.robot_to_right_cam)
    left_cam = sensors.PhotonCamCustom(config.left_cam_name, constants.robot_to_left_cam)
    cam_controller = sensors.PhotonController([left_cam, right_cam])




class PowerDistribution:
    pass


class Field:
    odometry = sensors.FieldOdometry(Robot.drivetrain, Sensors.cam_controller, constants.field_width, constants.field_length)
    field_constants = FieldConstants()
    reef_face = ReefFace
    branch = Branch
    reef_height = ReefHeight
    reef = Reef
    barge = Barge
    staging_positions = StagingPositions
    coral_station = CoralStation
    processor = Processor
    nt_reporter = NT_Updater("Field")

    @staticmethod
    def flip_poses():
        #print("Flipping Pos")
        flip_poses()

    @staticmethod
    def update_field_table():
        #print("Updating Table")
        update_table(Field.nt_reporter, False)
