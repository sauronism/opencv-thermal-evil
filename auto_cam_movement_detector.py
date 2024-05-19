"""
Created on Thu Jul 16 22:19:02 2020

@author: nagaraj
"""

import numpy as np
import math

import cv2

def find_cam_movement_between_frames(frame1, frame2):
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))

    # Take first frame and find corners in it
    frame1_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    # p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

    # Create a mask image for drawing purposes
    mask = np.zeros_like(frame1)
    mask_features = np.zeros_like(frame1_gray)
    mask_features[:, 0:20] = 1
    mask_features[:, 620:640] = 1

    # params for ShiTomasi corner detection
    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=3,
                          blockSize=7,
                          mask=mask_features)

    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


    def init_new_features(gray_frame):
        corners = cv2.goodFeaturesToTrack(gray_frame, **feature_params)
        return corners

    def calculateDistance(x1, y1, x2, y2):
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return dist

    corners = init_new_features(frame1_gray)

    frame_gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(frame1_gray, frame_gray, corners, None, **lk_params)
    good_new = p1
    good_old = corners

    total_distance = 0
    total_corner_points_found = 0

    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()

        distance = calculateDistance(a, b, c, d)

        total_distance += distance
        total_corner_points_found += 1

        p1 = (int(a), int(b))
        p2 = (int(c), int(d))

        frame1_gray = frame_gray.copy()
        corners = good_new.reshape(-1, 1, 2)
        mask = cv2.line(mask, p1, p2, color[i].tolist(), 2)
        frame2 = cv2.circle(frame2, p1, 5, color[i].tolist(), -1)

    avg_distance = total_distance // total_corner_points_found

    img = cv2.add(frame2, mask)

    text = "avg pixel distance of {avg_distance} pixels"
    utills.plant_text_bottom(img, text)
    cv2.imshow('distance_calc', img)

    return
