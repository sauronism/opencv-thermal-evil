from dataclasses import dataclass
from enum import Enum, StrEnum
from time import sleep
from typing import Union, Iterable, List

import cv2

from utills import draw_moving_contours, mark_target_contour, \
    is_target_in_circle, plant_state_name_in_frame, draw_light_beam, Vector, Contour


BEAM_RADIUS = 42
MIN_AREA_TO_CONSIDER = 7
MAX_AREA_TO_CONSIDER = 200

COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

ARROW_THICKNESS = 2

CIRCLE_THICKNESS = 2
FULL_SHAPE_THICKNESS = -1


def is_frame_in_movement(frame_contours: Iterable[Contour], moving_are_th: float) -> bool:
    area_in_movement = sum([c.area for c in frame_contours])
    return area_in_movement > moving_are_th


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    FRAME_X: int
    FRAME_Y: int

    BEAM_CENTER_POINT: Vector

    fg_backgorund: cv2.BackgroundSubtractorMOG2

    frame: Union[None, cv2.typing.MatLike] = None
    moving_contours: Union[None, List[Contour]] = None

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

    def is_contour_in_frame(self, contour: Contour) -> bool:
        # TODO - add to determine logic.
        is_contour_in_frame = self.frame
        return True


    def is_cam_in_movement(self, update_frame=False):
        if update_frame:
            self.update_frame()
        return is_frame_in_movement(self.moving_contours, self.IN_MOVEMENT_TH)

    def close_eye(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def update_frame(self):
        ret, frame = self.cap.read()
        self.frame = frame

        fg_mask = self.fg_backgorund.apply(frame)
        th = cv2.threshold(fg_mask, 0, 100, cv2.THRESH_BINARY)[1]

        contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.moving_contours = sorted([Contour(c, self.BEAM_CENTER_POINT) for c in contours], key=lambda c: -c.area)
