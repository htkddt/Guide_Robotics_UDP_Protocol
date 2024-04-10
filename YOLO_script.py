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

# Load model
model = YOLO("D:/A_Project_DK-TDH/PyCharm_Project/Demo/runs/detect/train8/weights/best.pt")

# Load class
license_class = [0, 1, 2, 3, 4]

results_list = {}
frame_nr = -1
point_center = None
ang = None

# Load Extrinsic matrix
R, t = Cam2base.read_matrix()
T_cam2base = np.vstack(((np.hstack((R, t))), ([0, 0, 0, 1.0])))

background = cv2.imread('D:\\A_Project_DK-TDH\\PyCharm_Project\\Background.jpg')
background_gray = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

while True:
    start_time = time.time()

    ret, color_frame, depth_frame, _, _, _ = rs.get_frame_stream()

    if ret:
        tl_point = (100, 275)
        br_point = (450, 420)
        cv2.rectangle(color_frame, tl_point, br_point, (0, 0, 0), 1)
        cropped_frame = color_frame[tl_point[1]:br_point[1], tl_point[0]:br_point[0]]

        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        difference_frame = cv2.absdiff(gray, background_gray)
        binary_frame = cv2.inRange(difference_frame, 25, 255, cv2.THRESH_BINARY)
        contour_box, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Yolo
        frame_nr += 1
        results_list[frame_nr] = {}

        # Detect
        license_results = model(color_frame)[0]
        license_plates = []

        for result in license_results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if int(class_id) in license_class:
                license_plates.append([x1, y1, x2, y2, score, class_id])

        # Draw bounding boxes on the image
        for result in license_plates:
            x1, y1, x2, y2, score, class_id = result
            point_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

            if ((point_center[0] > tl_point[0]) & (point_center[0] < br_point[0]) &
                    (point_center[1] > tl_point[1]) & (point_center[1] < br_point[1])):

                # point_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                top_left = (int(x1), int(y1))
                bottom_right = (int(x2), int(y2))

                # cv2.circle(color_frame, point_center, 3, (0, 0, 0), -1)
                cv2.circle(color_frame, top_left, 3, (0, 0, 0), 2)
                cv2.circle(color_frame, bottom_right, 3, (0, 0, 0), 2)

                if int(class_id) == 0:
                    cv2.rectangle(color_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)

                    cv2.putText(color_frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

                    cv2.putText(color_frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

                elif int(class_id) == 1:
                    cv2.rectangle(color_frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 185, 75), 2)

                    cv2.putText(color_frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)

                    cv2.putText(color_frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)

                elif int(class_id) == 2:
                    cv2.rectangle(color_frame, (int(x1), int(y1)), (int(x2), int(y2)), (2, 88, 230), 2)

                    cv2.putText(color_frame, str(round(score, 2)), (int(x1 + 30), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)

                    cv2.putText(color_frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)

                elif int(class_id) == 3:
                    cv2.rectangle(color_frame, (int(x1), int(y1)), (int(x2), int(y2)), (167, 72, 217), 2)

                    cv2.putText(color_frame, str(round(score, 2)), (int(x1 + 45), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)

                    cv2.putText(color_frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)

                elif int(class_id) == 4:
                    cv2.rectangle(color_frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 39, 205), 2)

                    cv2.putText(color_frame, str(round(score, 2)), (int(x1 + 55), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)

                    cv2.putText(color_frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)

                for c in contour_box:
                    area = cv2.contourArea(c)
                    if area < 1000:
                        continue
                    else:
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        if (cX > top_left[0]) & (cX < bottom_right[0]) & (cY > top_left[1]) & (cY < bottom_right[1]):
                            ang, point_center = getOrientation(c, color_frame)
                            ang = (ang * 180 / math.pi) + 90.0

                            if ang > 90.0:
                                ang = ang - 180

                            print("Angle = " + str(ang))

        cv2.imshow("Color frame", color_frame)
        cv2.imshow("Binary frame", binary_frame)
        cv2.imshow("Cropped frame", cropped_frame)

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
