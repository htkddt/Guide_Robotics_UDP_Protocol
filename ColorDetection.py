import sys
import cv2
import numpy as np


def COLOR_objectdetection(frame):
    u = None
    v = None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([20, 255, 255])

    lower_green = np.array([120, 100, 100])
    upper_green = np.array([180, 255, 255])

    lower_yellow = np.array([60, 100, 100])
    upper_yellow = np.array([120, 255, 255])

    mask_red = cv2.inRange(hsv, lower_red, upper_red, cv2.THRESH_BINARY)
    mask_green = cv2.inRange(hsv, lower_green, upper_green, cv2.THRESH_BINARY)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow, cv2.THRESH_BINARY)

    mask = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_green), mask_yellow)

    contour_box, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contour_box:
        area = cv2.contourArea(c)

        if (area < 4000) or (area > 8000):
            cv2.drawContours(area, [c], 0, (0, 0, 0), -1)
        else:
            x, y, w, h = cv2.boundingRect(c)

            center_pixel = (x + (w // 2), y + (h // 2))
            u = center_pixel[0]
            v = center_pixel[1]

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

            cv2.putText(frame, "u=", (u + 30, v),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, str(int(u)), (u + 55, v),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "v=", (u + 30, v + 20),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, str(int(v)), (u + 55, v + 20),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)

    return frame, u, v
