import cv2
import numpy as np

import utills


def locate_image_inside_frame(frame, image_to_locate):
    w, h = image_to_locate.shape[1], image_to_locate.shape[0]

    res = cv2.matchTemplate(frame, image_to_locate, cv2.TM_CCOEFF_NORMED)

    threshold = 0.95
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
        return pt


def calc_change_in_pixels(frame_origin_point, frame_post_move, direction_vector):
    h = frame_origin_point.shape[0]
    w = frame_origin_point.shape[1]

    third_y = h // 3
    third_x = w // 3

    crop_img = frame_origin_point[third_y: 2 * third_y, third_x: 2 * third_x]

    cv2.imshow('base', frame_origin_point)
    cv2.imshow('base_mid_cropped', crop_img)

    pt = locate_image_inside_frame(frame_post_move, crop_img)

    if pt:
        found_x = pt[0]
        found_y = pt[1]

        pixels_per_x = found_x - third_x
        pixels_per_y = found_y - third_y
    else:
        pixels_per_x = None
        pixels_per_y = None

    if cv2.waitKey(1) & 0xFF == ord('q'):
        raise

    if direction_vector.x != 0:
        text = f"X {direction_vector.x} degrees -> moved {pixels_per_x} pixels."
        pixels_moved = pixels_per_x
    else:
        text = f"Y {direction_vector.y} degrees -> moved {pixels_per_y} pixels."
        pixels_moved = pixels_per_y

    utills.plant_text_bottom(frame_post_move, text)
    cv2.imshow('post_move_find', frame_post_move)

    return abs(pixels_moved) if pixels_moved else None
