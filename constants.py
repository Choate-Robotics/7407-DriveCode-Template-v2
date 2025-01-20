from pathplannerlib.config import RobotConfig
# fix config import soon
import math
from units.SI import (
    degrees_per_second__to__radians_per_second,
    feet_to_meters,
    inches_to_meters,
    meters,
    rotations_per_minute,
    degrees_to_radians,
    meters_per_second_squared,
)
from pathplannerlib.config import PIDConstants

#drivetrain
drivetrain_turn_gear_ratio: float = 150 / 7
drivetrain_wheel_gear_ratio: float = 5.9
track_width: meters = 19.75 * inches_to_meters #distance between the center of the wheels (front side)
track_length: meters = 18.25 * inches_to_meters #(left/right side)
drivetrain_length: float = 20 #length of one side of the robot, placeholder
bumper_thickness: float = 3.5
drivetrain_length_with_bumpers = drivetrain_length + (2 * bumper_thickness)
drivetrain_radius: float = 20.5 # TODO: measure this

drivetrain_move_motor_free_speed: rotations_per_minute = (
    6000 #6000 is the free speed RPM of the Kraken without FOC
)

drivetrain_wheel_diameter: meters = (
        4 * inches_to_meters
)  
 

drivetrain_max_vel:meters = (
    ((drivetrain_move_motor_free_speed / 60) / drivetrain_wheel_gear_ratio) * (drivetrain_wheel_diameter * math.pi)
    )
# drivetrain_max_vel = 17.7
drivetrain_max_accel: meters_per_second_squared = 0 # setting to 0 will set to default motor accel
drivetrain_max_angular_vel = 720 * degrees_per_second__to__radians_per_second
drivetrain_max_angular_accel = 720 * degrees_per_second__to__radians_per_second


# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (
    1 / (drivetrain_wheel_diameter * math.pi)
) * drivetrain_wheel_gear_ratio

auto_config = RobotConfig.fromGUISettings()
# to change AUTO PIDs
auto_translation_pid = PIDConstants(5.0, 0.0, 0.0)
auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)


# field
field_length = 17.548
field_width = 8.052