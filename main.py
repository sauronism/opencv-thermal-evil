from dataclasses import dataclass
from time import sleep
from typing import Optional

import cv2
import numpy as np

from controller_ext_socket import DMXSocket
from thermal_camera import ThermalEye

from utills import Contour, draw_cam_direction_on_frame

RIGHT_KEY = 63235  # Right

LEFT_KEY = 63234  # Left

UP_KEY = 63232  # Up

DOWN_KEY = 63233  # Down


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
        self.y = get_value_within_limits(self.y + delta_y, bottom=-28, top=25)


def get_user_input_normalized():
    x, y = 0, 0
    key_pressed = cv2.waitKeyEx(1)
    if key_pressed == RIGHT_KEY:  # Right
        x -= 1
    if key_pressed == LEFT_KEY:  # Left
        x += 1
    if key_pressed == UP_KEY:  # Up
        y += 1
    if key_pressed == DOWN_KEY:  # Down
        y -= 1

    return x, y


@dataclass
class SauronEyeStateMachine:
    is_manual: bool  # 'manual' / 'camera'

    goal_coordinate: Coordinate

    socket: Optional[DMXSocket] = None

    thermal_eye: Optional[ThermalEye] = None

    beam: int = 42  # 0 - 255
    motor_on: bool = True

    leds_on: bool = False
    smoke_on: bool = False

    _beam_speed = 10

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
        while cv2.waitKeyEx(1) in [ord('w'), ord('s')]:
            speed_delta = 0
            if cv2.waitKeyEx(1) == ord('w'):
                speed_delta += 1
            if cv2.waitKeyEx(1) == ord('s'):
                speed_delta -= 1

            # beam speed range is 0-255
            self._beam_speed = get_value_within_limits(value, 0, 255)
            self.send_dmx_instructions()

    def send_dmx_instructions(self):
        payload = {
            "m": 1 if self.motor_on else 0,
            "s": 1 if self.smoke_on else 0,
            "b": self.beam,
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
        while True:
            self.set_manual_control()

            if self.thermal_eye:
                ret, self.thermal_eye.frame = self.thermal_eye.cap.read()
            else:
                self.thermal_eye.frame = np.ones((255,255))

            self.update_dmx_directions()

            if self.thermal_eye and self.thermal_eye.frame is not None:
                cv2.imshow('frame', self.thermal_eye.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    def speed_beam_motor_keyboard_updates(self):
        self.set_speed_with_keyboard()
        self.set_beam_with_keyboard()

        if cv2.waitKeyEx(1) == ord('m'):
            self.motor_on_off()

    def update_dmx_directions(self):
        self.speed_beam_motor_keyboard_updates()
        target, x_delta, y_delta = None, 0, 0

        if self.thermal_eye:
            target: Contour = self.thermal_eye.search_ring_bearer(print_frame=False)
            if target:
                x_delta = target.x_direction
                y_delta = target.y_direction

        if self.is_manual:
            x_delta, y_delta = get_user_input_normalized()
            if x_delta or y_delta:
                self.update_goal_dmx_coords(x_delta, y_delta)
        elif target:
            self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)

        draw_cam_direction_on_frame(self, x_delta, y_delta)

    def get_change_speed_command(self):
        speed_delta = 0
        if cv2.waitKeyEx(1) == ord('w'):
            speed_delta += 1
        if cv2.waitKeyEx(1) == ord('s'):
            speed_delta -= 1
        return speed_delta

    def update_goal_dmx_coords(self, x_delta, y_delta):
        x_delta *= self.beam_speed
        y_delta *= self.beam_speed

        self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)
        self.send_dmx_instructions()

    def set_beam_with_keyboard(self):
        while cv2.waitKeyEx(1) in [ord('e'), ord('d')]:
            delta_beam = 0
            if cv2.waitKeyEx(1) == ord('e'):
                delta_beam += 1
            if cv2.waitKeyEx(1) == ord('d'):
                delta_beam -= 1

            if delta_beam:
                self.beam = get_value_within_limits(self.beam + delta_beam, bottom=0, top=66)
                self.send_dmx_instructions()

    def motor_on_off(self):
        self.motor_on = not self.motor_on
        self.send_dmx_instructions()
        while cv2.waitKeyEx(1) == ord('m'):
            sleep(0.1)

    def set_speed_with_keyboard(self):
        speed_delta = self.get_change_speed_command()
        self.set_beam_speed(self.beam_speed + speed_delta)

    def set_manual_control(self):
        if cv2.waitKeyEx(1) == ord('p'):
            self.is_manual = not self.is_manual
            self.send_dmx_instructions()


if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeStateMachine(
        is_manual=True,
        goal_coordinate=Coordinate(90, 0),
        socket=dmx_socket,
        thermal_eye=thermal_eye
    )
    try:
        sauron.do_evil()
    except Exception as e:
        print(e)
    finally:
        dmx_socket.terminate_connection()
        thermal_eye.close_eye()
