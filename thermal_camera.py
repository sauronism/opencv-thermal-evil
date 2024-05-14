from dataclasses import dataclass

import cv2

from thermal_camera_static_view import draw_moving_contours, draw_vector_to_contour, \
    is_target_in_circle, plant_state_name_in_frame, draw_light_beam, Point, Contour

BEAM_RADIUS = 42
MIN_AREA_TO_CONSIDER = 3

COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

ARROW_THICKNESS = 2

CIRCLE_THICKNESS = 2
FULL_SHAPE_THICKNESS = -1


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    FRAME_X: int
    FRAME_Y: int

    BEAM_CENTER_POINT: Point

    fg_backgorund: cv2.BackgroundSubtractorMOG2

    def __init__(self, video_input):
        self.cap = cv2.VideoCapture(video_input)
        self.FRAME_W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.FRAME_H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.BEAM_CENTER_POINT = Point(x=self.FRAME_W // 2, y=self.FRAME_H // 2)

        self.FRAME_TOTAL_AREA = self.FRAME_W * self.FRAME_H
        self.IN_MOVEMENT_TH = self.FRAME_TOTAL_AREA // 4

        self.fg_backgorund = cv2.createBackgroundSubtractorMOG2(history=2)

    def find_closest_target(self, contours):
        center = self.BEAM_CENTER_POINT

        closest = None
        closest_distance = None

        for c in contours:
            c = c.obj
            x, y, w, h = cv2.boundingRect(c)

            contour_center_point = Point(x=x + w // 2, y=y + h // 2)

            distance = center.distance(contour_center_point)
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

        contours = sorted([Contour(c, center_point=self.BEAM_CENTER_POINT) for c in contours], key=lambda c: -c.area)

        area_in_movement = sum([c.area for c in contours])

        is_camera_in_movement = area_in_movement > self.IN_MOVEMENT_TH

        # filter small movements
        filtered_contours = [c for c in contours if c.area > MIN_AREA_TO_CONSIDER]

        if is_camera_in_movement:
            # camera in movement state
            state = 'IN MOVEMENT'
        else:
            state = 'SEARCHING THE RING'
            draw_light_beam(frame)

            contours_sorted = filtered_contours[:3]

            draw_moving_contours(frame, contours_sorted)

            target = self.find_closest_target(contours_sorted)

            draw_vector_to_contour(self.BEAM_CENTER_POINT, target)

            if is_target_in_circle(frame, target):
                state = "FOUND HOBBIT - LIGHT THE BEAM"

        if state:
            plant_state_name_in_frame(frame, state)

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
