import cv2
import numpy as np

# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, w, h):
    label = 'people'

    color = (255, 255, 255)

    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)

    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
