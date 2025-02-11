from units.SI import (
    radians,
    meters
)
from wpilib import AnalogEncoder
from toolkit.motors.ctre_motors import TalonConfig
import math
from pathplannerlib.config import PIDConstants


DEBUG_MODE: bool = True
# MAKE SURE TO MAKE THIS FALSE FOR COMPETITION
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
LOGGING: bool = True
LOG_OUT_LEVEL: int = 0
LOG_FILE_LEVEL: int = 1
# Levels are how much information is logged
# higher level = less information
# level 0 will log everything
# level 1 will log everything except debug
# and so on
# levels:
# 0 = All
# 1 = INFO
# 2 = WARNING
# 3 = ERROR
# 4 = SETUP
# anything else will log nothing
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

foc_active = False  #foc for TalonFX requires paid subscription

# Cameras
left_cam_name = "left_cam"
right_cam_name = "right_cam"

#Drivetrain
gyro_id: int = 13

front_left_move_id: int = 2
front_left_turn_id: int = 1
front_left_encoder_port: AnalogEncoder = AnalogEncoder(0)
front_left_encoder_zeroed_pos: float = 0.362
front_left_turn_inverted = False
front_left_move_inverted = False

front_right_move_id: int = 4
front_right_turn_id: int = 3
front_right_encoder_port: AnalogEncoder = AnalogEncoder(1)
front_right_encoder_zeroed_pos: float = 0.034
front_right_turn_inverted = False
front_right_move_inverted = False

back_left_move_id: int = 8
back_left_turn_id: int = 7
back_left_encoder_port: AnalogEncoder = AnalogEncoder(3)
back_left_encoder_zeroed_pos: float = 0.735
back_left_turn_inverted = False
back_left_move_inverted = False

back_right_move_id: int = 6
back_right_turn_id: int = 5
back_right_encoder_port: AnalogEncoder = AnalogEncoder(2)
back_right_encoder_zeroed_pos: float = 0.713
back_right_turn_inverted = False
back_right_move_inverted = False

driver_centric: bool = True
drivetrain_deadzone: float = 0.1
drivetrain_curve: float = 2.00000
drivetrain_zero: radians = math.radians(180)

# odometry
odometry_tag_distance_threshold: meters = 2.5

TURN_CONFIG = TalonConfig(
    8, 0, 0.025, 0, 0, brake_mode=True
)

MOVE_CONFIG = TalonConfig(
    0.11,
    0,
    0,
    kF=0.25,
    kA=0.15,
    kV=0.12,
    brake_mode=True,
    current_limit=50,
)

auto_translation_pid = PIDConstants(6, 0.0, 0.1)
auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)

# TO CHANGE
period = 0.03

leds_id=9
leds_size=27
leds_spacing=1/120.0
leds_speed=5
leds_brightness = 128
leds_saturation = 255
