from dataclasses import dataclass
from time import sleep
from typing import Optional

import cv2

from controller_ext_socket import DMXSocket
from thermal_camera import ThermalEye

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
    x, y, speed_delta = 0, 0, 0
    if keyboard.is_pressed('right'):
        x += 1
    if keyboard.is_pressed('left'):
        x -= 1
    if keyboard.is_pressed('up'):
        y += 1
    if keyboard.is_pressed('down'):
        y -= 1
    if keyboard.is_pressed('w'):
        speed_delta += 1
    if keyboard.is_pressed('s'):
        speed_delta -= 1

    return x, y, speed_delta


@dataclass
class SauronEyeStateMachine:
    operation_type: str  # 'manual' / 'camera'

    goal_coordinate: Coordinate

    socket: Optional[DMXSocket] = None

    thermal_eye: Optional[ThermalEye] = None

    beam_on: bool = True
    motor_on: bool = True

    leds_on: bool = False
    smoke_on: bool = False

    _beam_speed = 142

    @property
    def beam_x(self) -> float:
        # beam x limits are 0-179
        return self.goal_coordinate.x

    @property
    def beam_y(self) -> float:
        # beam x limits are 0-90
        return self.goal_coordinate.y


    @property
    def beam_speed(self) -> int:
        return self._beam_speed

    def set_beam_speed(self, value):
        # beam speed range is 0-255
        self._beam_speed = get_value_within_limits(value, 0, 255)
        self.send_dmx_instructions()

    def send_dmx_instructions(self):
        payload = {
            "m": 1 if self.motor_on else 0,
            "s": 1 if self.smoke_on else 0,
            "b": 1 if self.beam_on else 0,
            "x": self.beam_x,
            "y": self.beam_y,
            "v": self.beam_speed
        }
        # TODO - Establish and test communication with the DMX arduino.
        print(payload)

        if self.socket:
            self.socket.send_json(payload)

        sleep(0.1)
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

        # elif self.operation_type == 'camera':
        #     self.search_and_lock_eye()

    # def search_and_lock_eye(self):
    #     while True:
    #         x_delta, y_delta = self.thermal_eye.search_ring_bearer()
    #
    #         self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)
    #         print(self.goal_coordinate)
    #
    #         if keyboard.is_pressed('q'):
    #             break

    def control_dmx_with_keys(self):
        self.send_dmx_instructions()
        while True:
            x_delta, y_delta, speed_delta = get_user_input_normalized()
            if x_delta or y_delta:
                self.update_goal_dmx_coords(x_delta, y_delta)

            if speed_delta:
                self.set_speed_with_keyboard(speed_delta)

            if keyboard.is_pressed('b'):
                self.beam_on_off()

            if keyboard.is_pressed('m'):
                self.motor_on_off()

            if keyboard.is_pressed('q'):
                break

    def update_goal_dmx_coords(self, x_delta, y_delta):
        self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)
        self.send_dmx_instructions()

    def beam_on_off(self):
        self.beam_on = not self.beam_on
        self.send_dmx_instructions()
        while keyboard.is_pressed('b'):
            sleep(0.1)

    def motor_on_off(self):
        self.motor_on = not self.motor_on
        self.send_dmx_instructions()
        while keyboard.is_pressed('m'):
            sleep(0.1)

    def set_speed_with_keyboard(self, speed_delta):
        self.set_beam_speed(self.beam_speed + speed_delta)
        self.send_dmx_instructions()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # thermal_eye = get_thermal_eye_instance()
    # sauron = SauronEyeStateMachine(
    #     operation_type='camera',
    #     thermal_eye=thermal_eye
    # )
    dmx_socket = DMXSocket()
    sauron = SauronEyeStateMachine(
        operation_type='manual',
        goal_coordinate=Coordinate(142, 45),
        socket=dmx_socket
    )
    sauron.do_evil()
    dmx_socket.terminate_connection()
