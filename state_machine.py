import datetime
from dataclasses import dataclass, field
from time import sleep
from typing import Union, Optional

import cv2
import numpy as np

import utills
from auto_cam_movement_detector import find_cam_movement_between_frames
from controller_ext_socket import DMXSocket
from main import States, get_value_within_limits, PIXEL_DEGREES_MAPPER_FILE_PATH, \
    DEGREES_Y_MIN, DEGREES_Y_MAX, DEGREES_X_MIN, DEGREES_X_MAX, get_user_input_normalized, \
    MOVEMENT_VECTORS
from file_utills import save_json_file, get_json_from_file_if_exists
from frame_utills import calc_change_in_pixels
from thermal_camera import ThermalEye
from utills import Contour, Vector, draw_cam_direction_on_frame


@dataclass
class SauronEyeTowerStateMachine:
    is_manual: bool  # 'manual' / 'camera'

    state: Union[None, States] = None
    target: Union[None, Contour] = None

    use_auto_scale_file: bool = False
    pixel_degrees_mapper: Optional[dict] = None

    deg_coordinate: Vector = field(default_factory=Vector)
    goal_deg_coordinate: Vector = field(default_factory=Vector)

    socket: Optional[DMXSocket] = None

    thermal_eye: Optional[ThermalEye] = None

    beam: int = 0  # 0 - 255
    motor_on: bool = True

    leds_on: bool = False
    smoke_on: bool = False

    _beam_speed = 1

    def calculate_state(self, frame):
        starting_state = self.state
        current_target = self.target

        contours = self.get_moving_contours(frame)

        # Moving Camera States
        is_moving = self.is_frame_in_movement(contours)
        if is_moving:
            return States.MOVING_FRAME

        # filter small movements
        filtered_contours = [c for c in contours if c.area > MIN_AREA_TO_CONSIDER]
        if not filtered_contours:
            return States.SEARCH

        state = States.SEARCH
        draw_light_beam(frame)
        top_x = min(3, len(filtered_contours))

        contours_sorted = filtered_contours[:top_x]
        draw_moving_contours(frame, contours_sorted)

        largest_target = contours_sorted[0]
        closest_target = self.find_closest_target(contours_sorted)


        self.target = largest_target
        if is_target_in_circle(frame, self.target):
            return States.LOCKED

        if self.target:
            return States.FOUND_POSSIBLE_TARGET

        return state


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

    def send_dmx_instructions(self, print_return_payload=True):
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
            self.socket.send_json(print_return_payload=print_return_payload)

        return instruction_payload

    def do_evil(self):
        if self.use_auto_scale_file:
            mapper_dict = get_json_from_file_if_exists(PIXEL_DEGREES_MAPPER_FILE_PATH)
            self.pixel_degrees_mapper = self.auto_coordinate(mapper_dict)

        while True:
            self.send_dmx_instructions()
            # present frame
            frame = self.get_frame()
            cv2.imshow('frame', frame)

            key_pressed = cv2.waitKeyEx(1)

            # if key_pressed == ord('p'):
            #     self.programmer_mode(key_pressed)

            if key_pressed == ord('m'):
                self.set_manual_control(key_pressed, force_change=True)

            self.update_dmx_directions(key_pressed)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def auto_coordinate(self, mapper_dict):
        try:
            for y_degree in range(DEGREES_Y_MIN, DEGREES_Y_MAX):
                for x_degree in range(DEGREES_X_MIN, DEGREES_X_MAX):
                    point_key = (x_degree, y_degree)
                    point_mapping_dict = mapper_dict.get(point_key, {})
                    point_calculated = Vector(x_degree, y_degree)
                    mapper_dict[point_key] = self.map_pixel_degree_for_point(mapper_dict, point_calculated, point_mapping_dict)
        finally:
            save_json_file(PIXEL_DEGREES_MAPPER_FILE_PATH, mapper_dict)

        return mapper_dict

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


    def search_ring_bearer(self, print_frame=False):
        ret, frame = self.cap.read()
        self.frame = frame

        # Calculates target inside of state
        state = self.calculate_state(frame)

        # Draw Beam representation and plant state name on frame - Debugging purposes.
        utills.plant_state_name_in_frame(frame, state.value)
        utills.draw_light_beam(frame)

        # Waiting for a
        if state == States.MOVING_FRAME:
            return None

        if self.target:
            mark_target_contour(frame, self.BEAM_CENTER_POINT, self.target)

        target = self.target

        self.state = state
        if print_frame:
            cv2.imshow('frame', frame)

        return target


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

    def set_manual_control(self, key_pressed, force_change=False):
        if force_change or key_pressed == ord('m'):
            self.is_manual = not self.is_manual
            self.send_dmx_instructions()


    def map_pixel_degree_for_point(self, mapper_dict, point_calculated, point_mapping_dict):
        point_mapping_dict = point_mapping_dict or {}

        if len(point_mapping_dict.keys()) == 4:
            print(f'already calculated all directions for {point_mapping_dict}')
            return point_mapping_dict

        # move to origin point
        print(f'calcualting {point_calculated} calibration')
        self.move_to(point_calculated)
        frame_origin_point = self.get_frame(force_update=True)

        for direction_vector in MOVEMENT_VECTORS:
            if point_mapping_dict.get(direction_vector.as_tuple(), {}):
                print(f'Skipping due to prior calcs')
                continue

            print(f'--------------------------------------------')
            print(f'Calculating {point_calculated.as_tuple()} -> {direction_vector.as_tuple()}')


            point_after_movement = point_calculated + direction_vector
            self.move_to(point_after_movement)

            frame_post_move = self.get_frame(force_update=True)

            movement_degree_diff = 0
            calc_factor = 2  # normalizing 2 degree vectors

            diff_formulas_considered = 0

            # Calculate and Save pixel_diff
            pixel_diff = calc_change_in_pixels(frame_origin_point, frame_post_move, direction_vector)

            if pixel_diff is not None:
                movement_degree_diff += pixel_diff
                diff_formulas_considered += 1
            try:
                pixel_diff_2 = find_cam_movement_between_frames(frame_origin_point, frame_post_move)
                movement_degree_diff += pixel_diff_2
                diff_formulas_considered += 1
            except:
                movement_degree_diff += 0

            calc_factor *= diff_formulas_considered
            calc_factor = calc_factor or 1

            smoothed_distance = movement_degree_diff // calc_factor
            print(f'calculated {point_calculated.as_tuple()} -> {direction_vector.as_tuple()} = {smoothed_distance} Pixels')

            point_mapping_dict[direction_vector.as_tuple()] = smoothed_distance

            opposite_point = point_calculated - direction_vector
            opposite_dict = mapper_dict.get(opposite_point.as_tuple(), {})
            opposite_dict[(-direction_vector.x, -direction_vector.y)] = smoothed_distance
            print(f'calculated opposite Point {opposite_point.as_tuple()} -> {(-direction_vector.x, -direction_vector.y)} = {smoothed_distance} Pixels')
            print(f'--------------------------------------------')

        mapper_dict[point_calculated.as_tuple()] = point_mapping_dict
        return point_mapping_dict

    def move_to(self, point_calculated: Vector):
        self.goal_deg_coordinate = point_calculated

        for _ in range(3):
            self.send_dmx_instructions(print_return_payload=False)

        wait_for_move = datetime.timedelta(seconds=1.5)
        beginning = datetime.datetime.now()

        reached_timeout = datetime.datetime.now() - beginning > wait_for_move

        cam_in_movement = self.thermal_eye.is_cam_in_movement()
        while not cam_in_movement and not reached_timeout:
            print('Waiting movement')
            self.send_dmx_instructions(print_return_payload=False)
            frame = self.get_frame(force_update=True)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cam_in_movement = self.thermal_eye.is_cam_in_movement(frame=frame)
            reached_timeout = datetime.datetime.now() - beginning > wait_for_move

        beginning = datetime.datetime.now()
        while cam_in_movement and not reached_timeout:
            print('Camera Moving')
            self.send_dmx_instructions(print_return_payload=False)
            frame = self.get_frame(force_update=True)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cam_in_movement = self.thermal_eye.is_cam_in_movement(frame=frame)
            reached_timeout = datetime.datetime.now() - beginning > wait_for_move

        sleep(1)
        print(f'Camera reached {self.goal_deg_coordinate}')
