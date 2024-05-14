from dataclasses import dataclass
from enum import Enum, StrEnum
from typing import Union

import cv2

from utills import draw_moving_contours, mark_target_contour, \
    is_target_in_circle, plant_state_name_in_frame, draw_light_beam, Point, Contour


BEAM_RADIUS = 42
MIN_AREA_TO_CONSIDER = 3

COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

ARROW_THICKNESS = 2

CIRCLE_THICKNESS = 2
FULL_SHAPE_THICKNESS = -1


class States(StrEnum):
    MOVING_FRAME = 'IN MOVEMENT'
    SEARCH = 'SEARCHING THE RING'
    FOUND_RING = 'SEARCHING THE RING'
    FOUND_AND_LOCKED = 'FOUND HOBBIT - LIGHT THE BEAM'


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    FRAME_X: int
    FRAME_Y: int

    BEAM_CENTER_POINT: Point

    fg_backgorund: cv2.BackgroundSubtractorMOG2

    state: Union[None, States]

    def __init__(self, video_input):
        self.cap = cv2.VideoCapture(video_input)
        self.FRAME_W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.FRAME_H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.BEAM_CENTER_POINT = Point(x=self.FRAME_W // 2, y=self.FRAME_H // 2)

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


    def search_ring_bearer(self):
        state = None
        ret, frame = thermal_eye.cap.read()

        fg_mask = self.fg_backgorund.apply(frame)
        th = cv2.threshold(fg_mask, 0, 100, cv2.THRESH_BINARY)[1]
        contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contours = sorted([Contour(c) for c in contours], key=lambda c: -c.area)

        area_in_movement = sum([c.area for c in contours])

        is_camera_in_movement = area_in_movement > self.IN_MOVEMENT_TH

        # filter small movements
        filtered_contours = [c for c in contours if c.area > MIN_AREA_TO_CONSIDER]

        if is_camera_in_movement:
            # camera in movement state
            state = States.MOVING_FRAME
        else:
            state = States.SEARCH
            draw_light_beam(frame)

            contours_sorted = filtered_contours[:3]

            draw_moving_contours(frame, contours_sorted)

            target = self.find_closest_target(contours_sorted)

            if target:
                mark_target_contour(frame, self.BEAM_CENTER_POINT, target)
                state = States.FOUND_RING

            if is_target_in_circle(frame, target):
                mark_target_contour(frame, self.BEAM_CENTER_POINT, target)
                state = States.FOUND_AND_LOCKED

        plant_state_name_in_frame(frame, state.value)

        self.state = state
        cv2.imshow('frame', frame)

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
