from dataclasses import dataclass
from enum import Enum, StrEnum
from time import sleep
from typing import Union

import cv2

from utills import draw_moving_contours, mark_target_contour, \
    is_target_in_circle, plant_state_name_in_frame, draw_light_beam, Vector, Contour


BEAM_RADIUS = 42
MIN_AREA_TO_CONSIDER = 25

COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

ARROW_THICKNESS = 2

CIRCLE_THICKNESS = 2
FULL_SHAPE_THICKNESS = -1


class States(StrEnum):
    MOVING_FRAME = 'IN MOVEMENT'  # Waiting for movement to end and analyze a clean frame

    SEARCH = 'SEARCHING THE RING'  # Searching for largest moving object in frame

    FOUND_POSSIBLE_TARGET = 'Locking on Target'  # Searching for a new target

    LOST_TARGET = 'Lost target - looking for new'  # destination_point != current_point

    APPROACHING_TARGET = 'Moving to Target'  # destination_point != current_point

    SEARCHING_LOCKED_TARGET = 'Searching Locked Target'

    LOCKED = 'Locked on Ring!'  # until timeout or obj lost


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    FRAME_X: int
    FRAME_Y: int

    BEAM_CENTER_POINT: Vector

    fg_backgorund: cv2.BackgroundSubtractorMOG2

    state: Union[None, States] = None

    frame: Union[None, cv2.typing.MatLike] = None
    target: Union[None, Contour] = None

    def __init__(self, video_input):
        self.cap = cv2.VideoCapture(video_input)
        self.FRAME_W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.FRAME_H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.BEAM_CENTER_POINT = Vector(x=self.FRAME_W // 2, y=self.FRAME_H // 2)

        self.FRAME_TOTAL_AREA = self.FRAME_W * self.FRAME_H
        self.IN_MOVEMENT_TH = self.FRAME_TOTAL_AREA // 4

        self.fg_backgorund = cv2.createBackgroundSubtractorMOG2(history=2)

    def find_closest_target(self, contours):
        if not contours:
            return None

        center = self.BEAM_CENTER_POINT

        closest = None
        closest_distance = None

        for c in contours:
            distance = center.distance(c.center_point)
            if closest_distance is None or distance < closest_distance:
                closest = c
                closest_distance = distance

        return closest

    def is_frame_in_movement(self, frame_contours):
        area_in_movement = sum([c.area for c in frame_contours])
        return area_in_movement > self.IN_MOVEMENT_TH

    def is_cam_in_movement(self, contours=None, frame=None):
        if frame is None:
            ret, frame = self.cap.read()

        contours = contours or self.get_moving_contours(frame)
        # draw_moving_contours(frame, contours)

        is_in_movement = self.is_frame_in_movement(contours)
        if is_in_movement:
            self.state = States.MOVING_FRAME
            plant_state_name_in_frame(frame, self.state.value)

        self.frame = frame

        return is_in_movement

    def search_ring_bearer(self, print_frame=False):
        ret, frame = self.cap.read()
        self.frame = frame

        # Calculates target inside of state
        state = self.calculate_state(frame)

        # Draw Beam representation and plant state name on frame - Debugging purposes.
        plant_state_name_in_frame(frame, state.value)
        draw_light_beam(frame)

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

    def calculate_state(self, frame):
        starting_state = self.state

        contours = self.get_moving_contours(frame)

        # Moving Camera States
        is_moving = self.is_frame_in_movement(contours)
        if is_moving:
            return States.MOVING_FRAME

        if is_moving:
            return States.APPROACHING_TARGET

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
            return States.FOUND_AND_LOCKED

        if self.target:
            return States.FOUND_RING

        return state

    def get_moving_contours(self, frame):
        fg_backgorund = self.fg_backgorund

        fg_mask = fg_backgorund.apply(frame)
        th = cv2.threshold(fg_mask, 0, 100, cv2.THRESH_BINARY)[1]
        contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = sorted([Contour(c, self.BEAM_CENTER_POINT) for c in contours], key=lambda c: -c.area)

        return contours

    def draw_cam_direction_on_frame(self):
        pass

    def close_eye(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    thermal_eye = ThermalEye(
        video_input=0
    )
    while True:
        thermal_eye.search_ring_bearer()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
