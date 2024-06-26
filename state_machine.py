import datetime
from dataclasses import dataclass, field
from enum import StrEnum
from random import randrange
from time import sleep
from typing import Union, Optional, List

import cv2
import numpy as np

import utills
from auto_cam_movement_detector import find_cam_movement_between_frames
from controller_ext_socket import DMXSocket
from eye_motor_ext import send_motor_instruction
from file_utills import save_json_file, get_json_from_file_if_exists, PIXEL_DEGREES_MAPPER_FILE_PATH
from frame_utills import calc_change_in_pixels
from thermal_camera import ThermalEye, MIN_AREA_TO_CONSIDER, MAX_AREA_TO_CONSIDER, BEAM_RADIUS
from utills import Contour, DegVector, draw_cam_direction_on_frame, get_value_within_limits

from utills import DEGREES_X_MIN, DEGREES_X_MAX, DEGREES_Y_MIN, DEGREES_Y_MAX

SHOW_EVERY_TIMEDELTA = datetime.timedelta(minutes=5)
AUTO_SHOW_TRANSITION = datetime.timedelta(seconds=1)

FORGET_TARGET_TIMEOUT = datetime.timedelta(seconds=10)


class States(StrEnum):
    CALIBRATING = 'CALIBRATING'

    MOVING_TO_RANDOM_POINT = 'Moving to Random Point'

    MOVING_FRAME = 'IN MOVEMENT'  # Waiting for movement to end and analyze a clean frame

    SEARCH = 'SEARCHING THE RING'  # Searching for largest moving object in frame

    FOUND_POSSIBLE_TARGET = 'Found Target'  # Searching for a new target

    LOST_TARGET = 'Lost Target'

    CONFIRMING_LOCATIONS = 'Found Target - Validating Location'

    APPROACHING_TARGET = 'Moving to Target'  # destination_point != current_point

    SEARCHING_EXISTING_TARGET = 'Searching EXISTING Target'

    LOCKED = 'Locked on Ring! Following'  # until timeout or obj lost

    RE_LOCKING = 'Relocating Ring Bearer...'


def is_within_beam_limits(point: DegVector):
    if point.x > DEGREES_X_MAX or point.x < DEGREES_X_MIN or point.y > DEGREES_Y_MAX or point.y < DEGREES_Y_MIN:
        return False
    return True


@dataclass
class SauronEyeTowerStateMachine:
    is_manual: bool  # 'manual' / 'camera'

    state: Union[None, States] = None

    target: Union[None, Contour] = None

    pixel_degrees_mapper: Optional[dict] = None

    deg_coordinate: DegVector = field(default_factory=DegVector)
    goal_deg_coordinate: DegVector = field(default_factory=DegVector)

    socket: Optional[DMXSocket] = None

    thermal_eye: Optional[ThermalEye] = None

    frames_locked: int = 0

    latest_locked_state: Optional[datetime.datetime] = None
    last_automated_show: Optional[datetime.datetime] = None

    largest_target: Union[None, Contour] = None
    closest_target: Union[None, Contour] = None
    all_possible_targets: Optional[List[Contour]] = None

    beam: int = 0  # 0 - 255
    motor_on: bool = True

    search_radius: Optional[int] = None

    _beam_speed = 1

    def calculate_state(self, frame=None):
        if frame is None:
            frame = self.update_frame()

        now = datetime.datetime.now()

        has_target_state = self.state in [States.SEARCHING_EXISTING_TARGET, States.LOCKED]
        is_locked = self.state in [States.LOCKED, States.RE_LOCKING]

        self.search_radius, time_since_locked_on_target = 42_000, None
        if is_locked and self.latest_locked_state is not None:
            time_since_locked_on_target = now - self.latest_locked_state
            self.search_radius = int(BEAM_RADIUS + time_since_locked_on_target.total_seconds() * 20)

        self.target = None
        self.largest_target = None
        self.closest_target = None
        self.all_possible_targets = None

        # Moving Camera States
        is_moving = self.thermal_eye.is_cam_in_movement()
        if is_moving:
            return self.state

        # filter small movements
        filtered_contours = [c for c in self.thermal_eye.moving_contours
                             if c.get_abs_degree_location(self.deg_coordinate).is_inside_border
                             and (MIN_AREA_TO_CONSIDER < c.area < MAX_AREA_TO_CONSIDER)
                             and c.distance_from_center < self.search_radius]

        top_x = min(3, len(filtered_contours))
        self.all_possible_targets = filtered_contours[:top_x]

        if not self.all_possible_targets:
            if not is_locked:
                self.state = States.SEARCH
            elif is_locked and now - self.latest_locked_state > FORGET_TARGET_TIMEOUT:
                self.state = States.LOST_TARGET
            elif is_locked:
                self.state = States.RE_LOCKING

            return self.state

        self.largest_target = self.all_possible_targets[0]
        self.closest_target = self.thermal_eye.find_closest_target(self.all_possible_targets)

        if self.closest_target:
            self.target = self.closest_target

        is_target_in_beam = utills.is_target_in_circle(frame, self.closest_target)
        # is_target_in_frame = self.thermal_eye.is_contour_in_frame(self.target)

        if is_target_in_beam and self.frames_locked > 3:
            self.state = States.LOCKED
            self.latest_locked_state = now
        elif is_target_in_beam:
            self.frames_locked += 1
        else:
            self.frames_locked = 0

        if self.target and self.state == States.SEARCH:
            self.state = States.FOUND_POSSIBLE_TARGET

        return self.state

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

    def send_updated_state_signals(self, print_return_payload=True):
        instruction_payload = {
            "b": 10 if self.beam else 0,  # for safety
            "x": self.beam_x,
            "y": self.beam_y,
            "v": self.beam_speed
        }

        if self.socket:
            self.socket.instruction_payload = instruction_payload
            self.socket.send_json(print_return_payload=print_return_payload)

        # send_motor_instruction(self.motor_on, self.deg_coordinate.x)

        return instruction_payload

    def do_evil(self):
        self.set_beam_speed(1)
        while True:
            self.send_updated_state_signals()

            if (self.state != States.LOCKED and self.last_automated_show and
                    self.last_automated_show > datetime.datetime.now() - SHOW_EVERY_TIMEDELTA):
                self.run_automated_led_show(min_to_run=1)

            # present frame
            frame = self.update_frame()

            # Calculates target inside of state
            self.state = self.calculate_state(frame)

            frame, key_pressed = self.present_debug_frame(frame)

            target_deg_point = None
            if self.target:
                target_deg_point = self.target.get_abs_degree_location(self.deg_coordinate)

            if self.is_manual:
                self.update_dmx_directions(key_pressed)
            elif self.state in [States.FOUND_POSSIBLE_TARGET, States.SEARCHING_EXISTING_TARGET,
                                States.LOCKED, States.RE_LOCKING] and target_deg_point:
                is_locked = self.state in [States.LOCKED, States.RE_LOCKING]

                if self.target.distance_from_center < BEAM_RADIUS:
                    speed = 1
                elif self.target.distance_from_center < self.search_radius:
                    speed = 50
                else:
                    speed = 99

                self.set_beam_speed(speed)
                self.move_to(target_deg_point)
                if is_locked:
                    self.latest_locked_state = datetime.datetime.now()
            elif self.state == States.LOST_TARGET:
                self.set_beam_speed(1)
                self.go_to_random_spot_in_view()
                self.state = States.SEARCH

            # if key_pressed == ord('p'):
            #     self.programmer_mode(key_pressed)

            if key_pressed == ord('m'):
                self.set_manual_control(key_pressed, force_change=True)

            if key_pressed == ord('q'):
                break

    def present_debug_frame(self, frame=None, state=None):
        if frame is None:
            frame = self.get_frame()

        frame = self.draw_debugging_refs_on_frame(frame, state)
        cv2.imshow('frame', frame)
        key_pressed = cv2.waitKeyEx(1)

        return frame, key_pressed

    def auto_coordinate(self, mapper_dict):
        try:
            for y_degree in range(DEGREES_Y_MIN, DEGREES_Y_MAX):
                for x_degree in range(DEGREES_X_MIN, DEGREES_X_MAX):
                    point_key = (x_degree, y_degree)
                    point_mapping_dict = mapper_dict.get(point_key, {})
                    point_calculated = DegVector(x_degree, y_degree)
                    mapper_dict[point_key] = self.map_pixel_degree_for_point(mapper_dict, point_calculated, point_mapping_dict)
        finally:
            save_json_file(PIXEL_DEGREES_MAPPER_FILE_PATH, mapper_dict)

        self.pixel_degrees_mapper = mapper_dict

        return mapper_dict

    def programmer_mode(self, key_pressed):
        while key_pressed != ord('f'):
            key_pressed = cv2.waitKeyEx(1)

            frame = self.get_frame(update_frame=True)
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

    def update_frame(self):
        if not self.thermal_eye:
            return

        self.thermal_eye.update_frame()
        return self.thermal_eye.frame

    def get_frame(self):
        if not self.thermal_eye:
            return np.ones((255, 255))

        return self.thermal_eye.frame

    def speed_beam_motor_keyboard_updates(self, key_pressed):
        self.set_speed_with_keyboard(key_pressed)
        self.set_beam_with_keyboard(key_pressed)

        if key_pressed == ord('n'):
            self.motor_on_off()

        self.send_updated_state_signals()

    def draw_debugging_refs_on_frame(self, frame, state):
        # Draw Beam representation and plant state name on frame - Debugging purposes.
        frame = utills.plant_state_name_in_frame(frame, state or self.state)
        frame = utills.draw_light_beam(frame)

        if self.state == States.MOVING_FRAME:
            return frame

        frame = utills.draw_moving_contours(frame, self.all_possible_targets)

        if self.target:
            frame = utills.mark_target_contour(frame, self.thermal_eye.BEAM_CENTER_POINT, self.target)

        if self.search_radius:
            frame = utills.draw_search_radius_circle(frame, self.search_radius)

        if self.state == States.LOCKED:
            time_sec_locked_state = (datetime.datetime.now() - self.latest_locked_state).total_seconds()
            frame = utills.plant_text_bottom(frame, text=f'LOCKED for {int(time_sec_locked_state)} seconds')

        return frame

    def update_dmx_directions(self, key_pressed=None):
        x_delta, y_delta = get_user_input_normalized(key_pressed)

        if x_delta or y_delta:
            self.update_goal_dmx_coords(x_delta, y_delta)
            self.send_updated_state_signals()

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
        self.send_updated_state_signals()

    def set_speed_with_keyboard(self, key_pressed):
        speed_delta = self.get_change_speed_command(key_pressed)
        self.set_beam_speed(self.beam_speed + speed_delta)

    def set_manual_control(self, key_pressed, force_change=False):
        if force_change or key_pressed == ord('m'):
            self.is_manual = not self.is_manual
            self.send_updated_state_signals()


    def map_pixel_degree_for_point(self, mapper_dict, point_calculated, point_mapping_dict):
        point_mapping_dict = point_mapping_dict or {}

        if len(point_mapping_dict.keys()) == 4:
            print(f'already calculated all directions for {point_mapping_dict}')
            return point_mapping_dict

        # move to origin point
        print(f'calcualting {point_calculated} calibration')
        self.move_to(point_calculated, state=States.CALIBRATING)
        frame_origin_point = self.update_frame()

        for direction_vector in MOVEMENT_VECTORS:
            if point_mapping_dict.get(direction_vector.as_tuple(), {}):
                print(f'Skipping due to prior calcs')
                continue

            print(f'--------------------------------------------')
            print(f'Calculating {point_calculated.as_tuple()} -> {direction_vector.as_tuple()}')

            point_after_movement = point_calculated + direction_vector
            self.move_to(point_after_movement, state=States.CALIBRATING)

            frame_post_move = self.update_frame()

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

    def move_to(self, point_calculated: DegVector, state: States = States.MOVING_FRAME):
        # limit coordinates to MIN MAX values
        point_calculated.x = get_value_within_limits(point_calculated.x, bottom=DEGREES_X_MIN, top=DEGREES_X_MAX)
        point_calculated.y = get_value_within_limits(point_calculated.y, bottom=DEGREES_Y_MIN, top=DEGREES_Y_MAX)

        self.goal_deg_coordinate = point_calculated

        wait_for_move = datetime.timedelta(seconds=5)
        beginning = datetime.datetime.now()

        reached_timeout = datetime.datetime.now() - beginning > wait_for_move

        cam_in_movement = self.thermal_eye.is_cam_in_movement
        while not cam_in_movement and not reached_timeout:
            print('Waiting movement')
            cam_in_movement, is_manual_break = self.send_instruction_and_check_if_cam_is_moving(state)

            frame, key_pressed = self.present_debug_frame(state=state)
            is_manual_break = key_pressed == ord('q')

            reached_timeout = datetime.datetime.now() - beginning > wait_for_move
            if is_manual_break or reached_timeout:
                break

            reached_timeout = datetime.datetime.now() - beginning > wait_for_move

        beginning = datetime.datetime.now()
        while cam_in_movement:
            print('Camera Moving')
            cam_in_movement = self.send_instruction_and_check_if_cam_is_moving(state)

            frame, key_pressed = self.present_debug_frame(state=state)
            is_manual_break = key_pressed == ord('q')

            reached_timeout = datetime.datetime.now() - beginning > wait_for_move
            if is_manual_break or reached_timeout:
                break

        self.deg_coordinate = point_calculated

        if state == States.APPROACHING_TARGET:
            self.state = States.SEARCHING_EXISTING_TARGET

        sleep(0.5)

        print(f'Camera reached {self.goal_deg_coordinate}')

    def send_instruction_and_check_if_cam_is_moving(self, state):
        self.send_updated_state_signals(print_return_payload=False)

        cam_in_movement = self.thermal_eye.is_cam_in_movement(update_frame=True)

        return cam_in_movement

    def run_automated_led_show(self, min_to_run: int = 1):
        print('starting automated show.')

        beginning_time = datetime.datetime.now()
        time_passed = datetime.datetime.now() - beginning_time

        self.last_automated_show = beginning_time

        self.motor_on = True
        self.set_beam_speed(99)
        while time_passed < datetime.timedelta(minutes=min_to_run):
            time_passed = datetime.datetime.now() - beginning_time
            # Light beam in second 10.
            beam_on = int(time_passed.total_seconds()) % 60 > 10
            self.beam = 42 if beam_on else 0
            self.go_to_random_spot_in_view()

        self.motor_on = False
        self.send_updated_state_signals()

    def keep_state_and_present_frames_for_timedelta(self, timedelta: datetime.timedelta, state: States):
        start = datetime.datetime.now()
        time_passed = datetime.datetime.now() - start
        while time_passed < timedelta:
            self.send_updated_state_signals()
            self.update_frame()
            time_until_move = timedelta - time_passed
            self.present_debug_frame(state=state + f' {time_until_move.total_seconds()} sec left')
            time_passed = datetime.datetime.now() - start

    def go_to_random_spot_in_view(self):
        rand_x = randrange(DEGREES_X_MIN, DEGREES_X_MAX)
        rand_y = randrange(DEGREES_Y_MIN, DEGREES_Y_MAX)
        random_spot = DegVector(rand_x, rand_y)

        self.move_to(random_spot, States.MOVING_TO_RANDOM_POINT)



MOVEMENT_VECTORS = [
    DegVector(1, 0),
    DegVector(0, 1),
]
RIGHT_KEY = 63235  # Right
LEFT_KEY = 63234  # Left
UP_KEY = 63232  # Up
DOWN_KEY = 63233  # Down


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
