import json
import os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

import utills
from controller_ext_socket import DMXSocket
from thermal_camera import ThermalEye

from utills import Contour, draw_cam_direction_on_frame, Vector

PIXEL_DEGREES_MAPPER_FILE_PATH = './pixel_degrees_mapping_file.json'


RIGHT_KEY = 63235  # Right

LEFT_KEY = 63234  # Left

UP_KEY = 63232  # Up

DOWN_KEY = 63233  # Down


DEGREES_X_MIN, DEGREES_X_MAX = (0, 179)
DEGREES_Y_MIN, DEGREES_Y_MAX = (-28, 0)

MOVEMENT_VECTORS = [
    Vector(-1, 0),
    Vector(1, 0),
    Vector(0, -1),
    Vector(0, 1),
]

def get_value_within_limits(value, bottom, top):
    if value < bottom:
        return bottom
    if value > top:
        return top
    return value


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


def get_json_from_file_if_exists(file_path):
    if not os.path.isfile(file_path):
        return {}

    try:
        with open(file_path) as file:
            pixel_degrees_mapper = json.load(file)
    except Exception as e:
        print(e)
        pixel_degrees_mapper = {}
    return pixel_degrees_mapper


def calc_change_in_pixels(frame_origin_point, frame_post_move):
    return 0


@dataclass
class SauronEyeStateMachine:
    is_manual: bool  # 'manual' / 'camera'

    use_auto_scale_file: bool = False
    pixel_degrees_mapper: Optional[dict] = None

    deg_coordinate: Vector = field(default_factory=lambda: Vector)
    goal_deg_coordinate: Vector = field(default_factory=Vector)

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
        return self.goal_deg_coordinate.x

    @property
    def beam_y(self) -> float:
        # beam x limits are 0-90
        return self.goal_deg_coordinate.y

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
        instruction_payload = {
            "m": 1 if self.motor_on else 0,
            "s": 1 if self.smoke_on else 0,
            "b": self.beam,
            "x": self.beam_x,
            "y": self.beam_y,
            "v": self.beam_speed
        }

        if self.socket:
            self.socket.instruction_payload = instruction_payload
            self.socket.send_json()

        return instruction_payload

    def do_evil(self):
        # if self.use_auto_scale_file and self.thermal_eye:
        #     self.pixel_degrees_mapper = get_json_from_file_if_exists(PIXEL_DEGREES_MAPPER_FILE_PATH)
        #
        #     if not self.pixel_degrees_mapper:
        #         self.pixel_degrees_mapper = self.auto_reset_pixel_degrees_mapping()

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

        self.goal_deg_coordinate.x = get_value_within_limits(self.goal_deg_coordinate.x + x_delta,
                                                             bottom=DEGREES_X_MIN, top=DEGREES_X_MAX)
        self.goal_deg_coordinate.y = get_value_within_limits(self.goal_deg_coordinate.y + y_delta,
                                                             bottom=DEGREES_Y_MIN, top=DEGREES_Y_MAX)

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

    def auto_reset_pixel_degrees_mapping(self):
        mapper_dict = defaultdict(dict)

        for x_degree in range(DEGREES_X_MIN, DEGREES_X_MAX):
            for y_degree in range(DEGREES_Y_MIN, DEGREES_Y_MAX):
                point_calculated = Vector(x_degree, y_degree)
                mapper_dict = self.map_pixel_degree_for_point(mapper_dict, point_calculated)

        return mapper_dict

    def map_pixel_degree_for_point(self, mapper_dict, point_calculated):
        point_dict = mapper_dict[point_calculated.as_tuple()]

        for direction_vector in MOVEMENT_VECTORS:
            # move to origin point
            self.move_to(point_calculated)
            frame_origin_point = self.get_frame(force_update=True)

            # move to point_calculated

            point_after_movement = point_calculated + direction_vector
            self.move_to(point_after_movement)

            frame_post_move = self.get_frame(force_update=True)

            fg_backgorund = cv2.createBackgroundSubtractorMOG2(history=0)

            # Calculate and Save pixel_diff
            pixel_diff = calc_change_in_pixels(frame_origin_point, frame_post_move)
            point_dict[direction_vector.as_tuple()] = pixel_diff

        return mapper_dict

    def move_to(self, point_calculated: Vector):
        self.goal_deg_coordinate.x = point_calculated.x
        self.goal_deg_coordinate.y = point_calculated.y

        self.send_dmx_instructions()

        cam_in_movement = self.thermal_eye.is_cam_in_movement()
        while not cam_in_movement:
            print('Waiting movement')
            cam_in_movement = self.thermal_eye.is_cam_in_movement()

        while cam_in_movement:
            print('Camera Moving')
            cam_in_movement = self.thermal_eye.is_cam_in_movement()

        print(f'Camera reached {self.goal_deg_coordinate}')




if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeStateMachine(
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
