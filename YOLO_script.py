import sys
import cv2
import time
import imutils
import argparse
import math
import numpy as np

import Cam2base

from imutils import perspective
from imutils import contours

from ultralytics import YOLO
from YoloSegmentation import *
from YoloDetection import *
from RealsenseCamera import *


def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)

    angle = math.atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
    hypotenuse = math.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * math.cos(angle)
    q[1] = p[1] - scale * hypotenuse * math.sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * math.cos(angle + math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle + math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

    p[0] = q[0] + 9 * math.cos(angle - math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle - math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)


def getOrientation(pts, img):
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)

    for j in range(data_pts.shape[0]):
        data_pts[j, 0] = pts[j, 0, 0]
        data_pts[j, 1] = pts[j, 0, 1]

    # Perform PCA analysis
    mean = np.empty(0)
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)

    p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])

    drawAxis(img, cntr, p1, (255, 0, 0), 3)
    drawAxis(img, cntr, p2, (255, 255, 255), 7)

    angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

    if cntr[0] < p1[0]:
        angle = -angle

    return angle, cntr


# Load camera
rs = RealsenseCamera()

# Load model detect
yoloDetect = YoloDetection()

# Load model segment
yoloSegment = YoloSegmentation()

# Load model
# model = YOLO("D:/A_Project/PyCharm_Project/Application_Source/runs/detect/train9/weights/best.pt")

# Load class
# license_class = [0, 1, 2, 3, 4]

# results_list = {}
# frame_nr = -1
point_center = None
ang = None

# Load Extrinsic matrix
R, t = Cam2base.read_matrix()
T_cam2base = np.vstack(((np.hstack((R, t))), ([0, 0, 0, 1.0])))

# Cropp frame
min_X = 100
max_X = 300
min_Y = 275
max_Y = 420

width = 640
height = 480
black_frame = np.zeros((height, width), dtype=np.uint8)

while True:
    start_time = time.time()

    ret, color_frame, depth_frame, _, _, _ = rs.get_frame_stream()

    # tl_point = (min_X, min_Y)
    # br_point = (max_X, max_Y)
    # cv2.rectangle(color_frame, tl_point, br_point, (0, 0, 0), 1)

    if ret:
        binary_float32 = yoloSegment.getSegment(color_frame)
        color_frame, _, _, _, _, _ = yoloDetect.getObject(color_frame)

        if binary_float32 is not None:
            binary_normalized = cv2.normalize(binary_float32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            binary_frame = binary_normalized.astype(np.uint8)
            contour_box, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contour_box:
                area = cv2.contourArea(c)
                if area < 1000:
                    continue
                else:
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # if (cX > top_left[0]) & (cX < bottom_right[0]) & (cY > top_left[1]) & (cY < bottom_right[1]):
                    #     ang, _ = getOrientation(c, color_frame)
                    #     ang = (ang * 180 / math.pi) + 90.0
                    #
                    #     if ang > 90.0:
                    #         ang = ang - 180
                    #
                    #     print("Angle = " + str(ang))

                    ang, _ = getOrientation(c, color_frame)
                    ang = (ang * 180 / math.pi) + 90.0

                    if ang > 90.0:
                        ang = ang - 180

                    print("Angle = " + str(ang))

            cv2.imshow("Binary frame", binary_frame)
        else:
            cv2.imshow("Binary frame", black_frame)

    cv2.imshow("Color frame", color_frame)
    end_time_1 = time.time()

    print("FPS model = " + str(1 / (end_time_1 - start_time)) + " (Hz)")

    if point_center is not None:
        Xc, Yc, Zc, _ = rs.pixel_to_point(point_center[0], point_center[1])
        Wc = np.array([[Xc],
                       [Yc],
                       [Zc],
                       [1.0]])
        Wr = T_cam2base.dot(Wc)
        print("Wr = \n" + str(Wr) + '\n'
              "Yaw = " + str(ang))

        end_time_2 = time.time()

        print("FPS process = " + str(1 / (end_time_2 - start_time)) + " (Hz)")

    key = cv2.waitKey(1)
    if key == 27:  # ESC button
        break
