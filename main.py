from dataclasses import dataclass
from time import sleep
from typing import Optional

from thermal_camera import ThermalEye, get_thermal_eye_instance

import keyboard  # using module keyboard

def get_value_within_limits(value, bottom, top):
    if value < bottom:
        return bottom
    if value > top:
        return top
    return value


@dataclass
class Coordinate:
    x: float
    y: float

    def __str__(self):
        return f"Goal Coordinate ({self.x}, {self.y})"

    def update_goal_coordinate(self, delta_x, delta_y):
        # limit x, y coords
        self.x = get_value_within_limits(self.x + delta_x, bottom=0, top=179)
        self.y = get_value_within_limits(self.y + delta_y, bottom=0, top=90)


def get_user_input_normalized():
    x, y = 0, 0
    if keyboard.is_pressed('right'):
        x += 1
    if keyboard.is_pressed('left'):
        x -= 1
    if keyboard.is_pressed('up'):
        y += 1
    if keyboard.is_pressed('down'):
        y -= 1
    if x or y:
        sleep(0.05)
    return x, y


@dataclass
class SauronEyeStateMachine:
    operation_type: str  # 'manual' / 'camera'

    goal_coordinate: Coordinate
    thermal_eye: Optional[ThermalEye] = None

    beam_on: bool = False
    motor_on: bool = False

    leds_on: bool = False
    smoke_on: bool = False


    @property
    def beam_x(self) -> float:
        # beam x limits are 0-179
        return self.goal_coordinate.x

    @property
    def beam_y(self) -> float:
        # beam x limits are 0-90
        return self.goal_coordinate.y

    #
    @property
    def beam_speed(self) -> int:
        # beam speed range is 0-255
        return 42

    def send_dmx_instructions(self):
        payload = {
            "motor_on": self.motor_on,
            "smoke_on": self.smoke_on,
            "beam_on": self.smoke_on,
            "beam_x": self.beam_x,
            "beam_y": self.beam_y,
            "beam_speed": self.beam_speed
        }
        # TODO - Establish and test communication with the DMX arduino.
        return payload

    def send_led_eye_instructions(self):
        # TODO - Establish and test communication with Wifi to LED teensie.
        payload = {
            "leds_on": self.leds_on,
            "beam_x": self.beam_x
        }
        return payload

    def do_evil(self):
        if self.operation_type == 'manual':
            self.control_dmx_with_keys()

        elif self.operation_type == 'camera':
            self.search_and_lock_eye()

    def search_and_lock_eye(self):
        while True:
            x_delta, y_delta = self.thermal_eye.search_ring_bearer()

            self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)
            print(self.goal_coordinate)

            if keyboard.is_pressed('q'):
                break

    def control_dmx_with_keys(self):
        while True:
            x_delta, y_delta = get_user_input_normalized()
            self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)
            print(self.goal_coordinate)

            if keyboard.is_pressed('q'):
                break


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # thermal_eye = get_thermal_eye_instance()
    # sauron = SauronEyeStateMachine(
    #     operation_type='camera',
    #     thermal_eye=thermal_eye
    # )
    
    sauron = SauronEyeStateMachine(
        operation_type='manual',
        goal_coordinate=Coordinate(90, 45)
    )
    sauron.do_evil()
