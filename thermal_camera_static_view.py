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

if __name__ == "__main__":
    # thermal_eye = get_thermal_eye_instance_video_test()
    thermal_eye = get_thermal_eye_instance_thermal_cam(0)

    backgroundObject = cv2.BackgroundSubtractorMOG2()
    # backgroundObject.setHistory(2)

    kernel = np.ones((3, 3), np.uint8)

    while True:
        ret, frame = thermal_eye.cap.read()

        if not ret:
            print('no cap video.')
            break

        fg_mask = backgroundObject.apply(frame)

        # Find contours
        contours, hierarchy = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(contours)
        frame_ct = cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
        # Display the resulting frame
        cv2.imshow('Frame_final', frame_ct)

        # cv2.threshold(fgmask, 20, 255, cv2.THRESH_BINARY)
        # fgmask = cv2.erode(fgmask, kernel, iterations=1)

        # cv2.imshow("frame", frame)
        # cv2.imshow("fgmask", fgmask)

        if cv2.waitKey(33) == ord('q'):
            break

