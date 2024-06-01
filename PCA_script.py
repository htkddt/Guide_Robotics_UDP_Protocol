# Huỳnh Tuấn Kiệt - 2010364
import sys
import cv2
import imutils
import argparse
import math
import numpy as np

from imutils import perspective
from imutils import contours

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

    cv2.circle(img, cntr, 3, (255, 0, 255), -1)
    p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
    drawAxis(img, cntr, p1, (255, 0, 0), 3)
    drawAxis(img, cntr, p2, (255, 255, 255), 7)

    angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

    return angle, cntr


image = cv2.imread('D:\\A_Project\\PyCharm_Project\\PCA_Images\\image_(9).jpg')

# Convert image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Load Background
background = cv2.imread('D:\\A_Project\\PyCharm_Project\\Background.jpg')
background_gray = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

# Background Subtraction
difference_frame = cv2.absdiff(gray, background_gray)

# Convert image to binary
binary = cv2.inRange(difference_frame, 25, 255, cv2.THRESH_BINARY)

# Find contour
contour_box, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
ang = 0

for i, c in enumerate(contour_box):
    area = cv2.contourArea(c)
    if area < 5000:
        continue
    else:
        cv2.drawContours(image, contour_box, i, (0, 255, 0), 2)
        ang, _ = getOrientation(c, image)
        print(str(ang))

print("Angle = " + str(ang))

cv2.imshow('Image', image)
cv2.imshow('Binary', binary)
cv2.waitKey(0)
cv2.destroyAllWindows()
