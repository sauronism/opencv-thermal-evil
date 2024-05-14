import math
from dataclasses import dataclass
from functools import cached_property
from typing import Sequence, Self

import numpy as np
import cv2


@dataclass
class Point:
    x: int
    y: int

    def distance(self, other: Self):
        dx2 = (self.x - other.x) ** 2
        dy2 = (self.y - other.y) ** 2
        return int(math.sqrt(dx2 + dy2))

    def as_tuple(self):
        return self.x, self.y


@dataclass
class Contour:
    obj: tuple[Sequence[cv2.UMat], cv2.UMat]

    x: int
    y: int
    w: int
    h: int

    area: int  # pixels area
    center_point: Point  # distance from center

    def __init__(self, c):
        self.obj = c

        self.area = int(cv2.contourArea(c))

        self.x, self.y, self.w, self.h = cv2.boundingRect(c)

    @cached_property
    def center_point(self):
        return Point(x=int(self.x + self.w // 2), y=int(self.y + self.h // 2))

    @cached_property
    def top_left_point(self):
        return Point(x=self.x, y=self.y)

    @cached_property
    def bottom_right_point(self):
        return Point(x=self.x + self.w, y=self.y + self.h)

    def get_direction_vector(self):
        pass


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    last_frames = []
    frame_count = 0

    def show_frame(self):
        # captures a single frame
        pass

    def close_eye(self):
        self.cap.release()
        cv2.destroyAllWindows()


def get_thermal_eye_instance_video_test():
    cap = cv2.VideoCapture('thermal_video_test.mp4')
    return ThermalEye(
        cap=cap
    )


def get_thermal_eye_instance_thermal_cam(device_id):
    cap = cv2.VideoCapture(device_id)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))

    return ThermalEye(
        cap=cap
    )



def detect_motion(frame, last_mean):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    result = np.abs(np.mean(gray) - last_mean)
    last_mean = np.mean(gray)

    print(result)
    if result > 0.3:
        print("Motion detected!")
        print("Started recording.")
        return True, last_mean
    return False, last_mean


def plant_state_name_in_frame(frame, state_name):
    # setup text
    font = cv2.FONT_HERSHEY_SIMPLEX

    # get boundary of this text
    text_size = cv2.getTextSize(state_name, font, 1, 2)[0]

    # get coords based on boundary
    textX = (frame.shape[1] - text_size[0]) // 2
    textY = (frame.shape[0] + text_size[1]) // 5

    # add text centered on image
    cv2.putText(frame, state_name, (textX, textY), font, 1, (255, 255, 255), 2)


BEAM_RADIUS = 42
MIN_AREA_TO_CONSIDER = 3


COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

ARROW_THICKNESS = 2

CIRCLE_THICKNESS = 2
FULL_SHAPE_THICKNESS = -1


def draw_light_beam(frame):
    center_of_circle = (frame.shape[1] // 2, frame.shape[0] // 2)

    cv2.circle(frame, center_of_circle, BEAM_RADIUS, COLOR_RED, CIRCLE_THICKNESS)


def draw_moving_contours(frame, contours):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour.obj)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        

def is_target_in_circle(frame, target_c: Contour):
    if not target_c:
        return False

    mask_frame = np.zeros(frame.shape)
    center_of_circle = (frame.shape[1] // 2, frame.shape[0] // 2)

    cv2.circle(mask_frame, center_of_circle, BEAM_RADIUS, COLOR_WHITE, FULL_SHAPE_THICKNESS)

    beam_mask_frame_sum = mask_frame.sum()

    cv2.rectangle(mask_frame,
                  target_c.top_left_point.as_tuple(),
                  target_c.bottom_right_point.as_tuple(),
                  COLOR_BLACK, FULL_SHAPE_THICKNESS)

    beam_and_target_frame_sum = mask_frame.sum()

    cv2.imshow('collision_mask_frame', mask_frame)

    return beam_mask_frame_sum != beam_and_target_frame_sum




def mark_target_contour(frame, center_point: Point, target_c: Contour):
    cv2.rectangle(frame, (target_c.x, target_c.y), (target_c.x + target_c.w, target_c.y + target_c.h), COLOR_BLACK, 2)

    contour_center = target_c.center_point
    cv2.arrowedLine(frame, center_point.as_tuple(), contour_center.as_tuple(),
                    COLOR_BLACK, ARROW_THICKNESS)
