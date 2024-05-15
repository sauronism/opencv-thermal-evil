import datetime
from dataclasses import dataclass
from time import sleep
from typing import Optional

import cv2
import numpy as np

import utills
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
        self.y = get_value_within_limits(self.y + delta_y, bottom=-28, top=0)


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

    @property
    def mode(self) -> int:
        return "manual" if self.is_manual else "auto"

    def set_beam_speed(self, value, key_pressed=None):
        speed_delta = 0
        if key_pressed == ord('w'):
            speed_delta += 1
        if key_pressed == ord('s'):
            speed_delta -= 1

        # beam speed range is 0-255
        self._beam_speed = get_value_within_limits(value + speed_delta, 0, 255)

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
            self.send_dmx_instructions()
            # present frame
            frame = self.get_frame()
            cv2.imshow('frame', frame)

            key_pressed = cv2.waitKeyEx(1)
            if key_pressed == ord('p'):
                self.programmer_mode(key_pressed)

            self.update_dmx_directions(key_pressed)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def programmer_mode(self, key_pressed):
        while key_pressed != ord('f'):
            key_pressed = cv2.waitKeyEx(1)

            frame = self.get_frame(force_update=True)
            utills.plant_state_name_in_frame(frame, 'programmer_mode')

            self.set_manual_control(key_pressed)
            self.speed_beam_motor_keyboard_updates(key_pressed)

            mode = "manual" if self.is_manual else "auto"

            setup_state_text = f'Mode: {mode}, ' \
                               f'Speed: {self.beam_speed}, ' \
                               f'Brightness: {self.beam}, ' \
                               f'Eye Motor: {self.motor_on}, ' \
                               f'' \
                               f'Press "f" to finish'

            utills.plant_text_bottom(frame, setup_state_text)

            cv2.imshow('frame', frame)

    def get_frame(self, force_update=False):
        if not self.thermal_eye:
            return np.ones((255, 255))

        if force_update or self.thermal_eye.frame is None:
            ret, frame = self.thermal_eye.cap.read()
            self.thermal_eye.frame = frame

        return self.thermal_eye.frame

    def speed_beam_motor_keyboard_updates(self, key_pressed):
        self.set_speed_with_keyboard(key_pressed)
        self.set_beam_with_keyboard(key_pressed)

        if key_pressed == ord('n'):
            self.motor_on_off()

        self.send_dmx_instructions()

    def update_dmx_directions(self, key_pressed=None):
        target, x_delta, y_delta = None, 0, 0

        if self.thermal_eye:
            target: Optional[Contour] = self.thermal_eye.search_ring_bearer(print_frame=False)

        if self.is_manual and key_pressed:
            x_delta, y_delta = get_user_input_normalized(key_pressed)
        elif target:
            self.set_beam_speed(1)
            x_delta = target.x_direction
            y_delta = target.y_direction

        if x_delta or y_delta:
            self.update_goal_dmx_coords(x_delta, y_delta)
            self.send_dmx_instructions()

        draw_cam_direction_on_frame(self, x_delta, y_delta)


    def get_change_speed_command(self, key_pressed):
        speed_delta = 0
        if key_pressed == ord('w'):
            speed_delta += 1
        if key_pressed == ord('s'):
            speed_delta -= 1
        return speed_delta

    def update_goal_dmx_coords(self, x_delta, y_delta):
        x_delta *= self.beam_speed
        y_delta *= self.beam_speed

        self.goal_coordinate.update_goal_coordinate(x_delta, y_delta)

    def set_beam_with_keyboard(self, key_pressed):
        delta_beam = 0
        if key_pressed == ord('e'):
            delta_beam += 1
        if key_pressed == ord('d'):
            delta_beam -= 1

        if delta_beam != 0:
            self.beam = get_value_within_limits(self.beam + delta_beam, bottom=0, top=66)

    def motor_on_off(self):
        self.motor_on = not self.motor_on
        self.send_dmx_instructions()

    def set_speed_with_keyboard(self, key_pressed):
        speed_delta = self.get_change_speed_command(key_pressed)
        self.set_beam_speed(self.beam_speed + speed_delta)

    def set_manual_control(self, key_pressed):
        if key_pressed == ord('m'):
            self.is_manual = not self.is_manual
            self.send_dmx_instructions()


if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeStateMachine(
        is_manual=True,
        goal_coordinate=Coordinate(110, -10),
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
