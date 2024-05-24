from enum import StrEnum
from pathlib import Path

from controller_ext_socket import DMXSocket
from state_machine import SauronEyeTowerStateMachine
from thermal_camera import ThermalEye

from utills import Vector

PIXEL_DEGREES_MAPPER_FILE_PATH = Path('./pixel_degrees_dict_file')


RIGHT_KEY = 63235  # Right

LEFT_KEY = 63234  # Left

UP_KEY = 63232  # Up

DOWN_KEY = 63233  # Down


DEGREES_X_MIN, DEGREES_X_MAX = (60, 120)
DEGREES_Y_MIN, DEGREES_Y_MAX = (-28, 0)


MOVEMENT_VECTORS = [
    Vector(1, 0),
    Vector(0, 1),
]


class States(StrEnum):
    MOVING_FRAME = 'IN MOVEMENT'  # Waiting for movement to end and analyze a clean frame

    SEARCH = 'SEARCHING THE RING'  # Searching for largest moving object in frame

    FOUND_POSSIBLE_TARGET = 'Locking on Target'  # Searching for a new target

    LOST_TARGET = 'Lost target - looking for new'  # destination_point != current_point

    APPROACHING_TARGET = 'Moving to Target'  # destination_point != current_point

    SEARCHING_LOCKED_TARGET = 'Searching Locked Target'

    LOCKED = 'Locked on Ring!'  # until timeout or obj lost


def get_value_within_limits(value, bottom, top):
    return min(max(value, bottom), top)


def get_user_input_normalized(key_pressed):
    x, y = 0, 0
    key_pressed = key_pressed
    if key_pressed == RIGHT_KEY:  # Right
        x -= 1
    if key_pressed == LEFT_KEY:  # Left
        x += 1
    if key_pressed == UP_KEY:  # Up
        y += 1
    if key_pressed == DOWN_KEY:  # Down
        y -= 1

    return x, y


if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeTowerStateMachine(
        is_manual=True,
        socket=dmx_socket,
        thermal_eye=thermal_eye,

        use_auto_scale_file=True,
    )
    try:
        sauron.do_evil()
    finally:
        dmx_socket.terminate_connection()
        thermal_eye.close_eye()
