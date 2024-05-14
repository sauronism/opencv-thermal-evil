import math
from dataclasses import dataclass

import numpy as np
import cv2

from open_cv_utills import draw_bounding_box


SAVE_LAST_FRAMES_AMOUNT = 5


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


def plant_state_name_in_frame(frame, state):
    # setup text
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = state

    # get boundary of this text
    text_size = cv2.getTextSize(text, font, 1, 2)[0]

    # get coords based on boundary
    textX = (frame.shape[1] - text_size[0]) // 2
    textY = (frame.shape[0] + text_size[1]) // 5

    # add text centered on image
    cv2.putText(frame, text, (textX, textY), font, 1, (255, 255, 255), 2)


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
    for contour, area in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


def find_closest_target(frame, contours):
    center_of_circle = (frame.shape[1] // 2, frame.shape[0] // 2)

    closest = None
    closest_distance = None
    
    for c, area in contours:
        x, y, w, h = cv2.boundingRect(c)
        
        dx2 = (center_of_circle[0] - x) ** 2
        dy2 = (center_of_circle[1] - y) ** 2
        
        distance = math.sqrt(dx2 + dy2)
        
        if closest_distance is None or distance < closest_distance:
            closest = c
            closest_distance = distance
    
    return closest
        

def is_target_in_circle(frame, target_c):
    mask_frame = np.zeros(frame.shape)
    center_of_circle = (frame.shape[1] // 2, frame.shape[0] // 2)

    cv2.circle(mask_frame, center_of_circle, BEAM_RADIUS, COLOR_WHITE, FULL_SHAPE_THICKNESS)

    beam_mask_frame_sum = mask_frame.sum()

    x, y, w, h = cv2.boundingRect(target_c)
    cv2.rectangle(mask_frame, (x, y), (x + w, y + h), COLOR_BLACK, FULL_SHAPE_THICKNESS)

    beam_and_target_frame_sum = mask_frame.sum()

    cv2.imshow('collision_mask_frame', mask_frame)

    return beam_mask_frame_sum != beam_and_target_frame_sum




def draw_vector_to_target(frame, target_c):
    x, y, w, h = cv2.boundingRect(target_c)
    cv2.rectangle(frame, (x, y), (x + w, y + h), COLOR_BLACK, 2)

    target_center = (x + (w // 2), y + (h // 2))

    center_of_circle = (frame.shape[1] // 2, frame.shape[0] // 2)

    cv2.arrowedLine(frame, center_of_circle, target_center,
                    COLOR_BLACK, ARROW_THICKNESS)


if __name__ == "__main__":
    # thermal_eye = get_thermal_eye_instance_video_test()
    thermal_eye = get_thermal_eye_instance_thermal_cam(0)

    cap = thermal_eye.cap

    FRAME_WIDTH = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    FRAME_HEIGHT = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    FRAME_TOTAL_AREA = FRAME_WIDTH * FRAME_HEIGHT
    IN_MOVEMENT_TH = FRAME_TOTAL_AREA // 2

    fg_backgorund = cv2.createBackgroundSubtractorMOG2(history=2)

    # choose codec according to format needed
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter('output.mp4', fourcc, 20.0, (FRAME_WIDTH, FRAME_HEIGHT))
    # count = 0

    while True:
        state = None
        ret, frame = thermal_eye.cap.read()

        fg_mask = fg_backgorund.apply(frame)
        th = cv2.threshold(fg_mask, 0, 100, cv2.THRESH_BINARY)[1]
        contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contour_size_tup = [(c, cv2.contourArea(c)) for c in contours]

        area_in_movement = sum([c[1] for c in contour_size_tup])
        is_camera_in_movement = area_in_movement > IN_MOVEMENT_TH

        # filter small movements
        filtered_contours = [c for c in contour_size_tup if c[1] > MIN_AREA_TO_CONSIDER]

        if is_camera_in_movement:
            # camera in movement state
            state = 'IN_MOVEMENT'
        else:
            state = 'SEARCHING THE RING'
            draw_light_beam(frame)

            contours_sorted = sorted(filtered_contours, key=lambda c: -c[1])[:3]

            draw_moving_contours(frame, contours_sorted)

            target = find_closest_target(frame, contours_sorted)
            
            draw_vector_to_target(frame, target)
            
            if is_target_in_circle(frame, target):
                state = "FOUND HOBBIT - LIGHT THE BEAM"

        if state:
            plant_state_name_in_frame(frame, state)

        cv2.imshow('frame', frame)

        # video_writer.write(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        # cv2.imwrite("frame_%d.tiff" % count, frame)  # save frame as JPEG file
        # count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    thermal_eye.cap.release()
    cv2.destroyAllWindows()
    # video_writer.release()
