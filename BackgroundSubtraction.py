# Huỳnh Tuấn Kiệt - 2010364
import math
import sys
import cv2
import imutils
import numpy as np
from imutils import perspective
from imutils import contours
from RealsenseCamera import *
# Test source

rs = RealsenseCamera()

background_gray = None

max_point = None

cnt = 0

while True:
    # Đọc khung ảnh từ camera RealSense
    ret, color_frame, _, _, _, _ = rs.get_frame_stream()

    if ret:
        if cnt < 50:
            cnt += 1
        else:
            if background_gray is None:
                background_gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

            else:
                # Chuyển đổi khung ảnh sang ảnh xám
                gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

                # Trừ ảnh: frame hiện tại - frame trước đó
                frame = cv2.absdiff(gray, background_gray)

                # Ngưỡng hóa ảnh
                binary = cv2.inRange(frame, 25, 255, cv2.THRESH_BINARY)

                # kernel = np.ones((5, 5), np.uint8)
                # binary = cv2.erode(binary, kernel, iterations=1)
                # binary = cv2.dilate(binary, kernel, iterations=1)

                contour_box, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for c in contour_box:
                    area = cv2.contourArea(c)
                    if area < 2000:
                        continue
                    else:
                        # Lấy minRect
                        box = cv2.minAreaRect(c)
                        # Lấy tọa độ các đỉnh của MinRect
                        point = cv2.boxPoints(box)

                        point_array = np.array(point, dtype="int")

                        point_list = perspective.order_points(point_array)
                        cv2.drawContours(color_frame, [point_list.astype("int")], -1, (0, 255, 0), 2)

                        (point_A, point_B, point_C, point_D) = point_list
                        point_M = (int((point_A[0] + point_B[0]) / 2), int((point_A[1] + point_B[1]) / 2))
                        point_N = (int((point_C[0] + point_D[0]) / 2), int((point_C[1] + point_D[1]) / 2))
                        point_Q = (int((point_A[0] + point_D[0]) / 2), int((point_A[1] + point_D[1]) / 2))
                        point_K = (int((point_B[0] + point_C[0]) / 2), int((point_B[1] + point_C[1]) / 2))

                        epsilon = 0.01 * cv2.arcLength(c, True)
                        approx = cv2.approxPolyDP(c, epsilon, True)
                        cv2.drawContours(color_frame, [approx], -1, (0, 255, 0), 2)

                        """
                        # Fit đường thẳng vào contour
                        (vx, vy, x, y) = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)

                        vx = float(vx[0])
                        vy = float(vy[0])
                        x = float(x[0])
                        y = float(y[0])

                        # Tính toán các điểm trên đường thẳng
                        lefty = int((-x * vy / vx) + y)
                        righty = int(((gray.shape[1] - x) * vy / vx) + y)

                        # Vẽ đường thẳng trên ảnh
                        # cv2.circle(color_frame, point_A, 5, (0, 0, 0), -1)
                        cv2.line(color_frame, (gray.shape[1], righty), (0, lefty), (0, 255, 255), 2)

                        # cv2.line(color_frame, point_A, point_C, (0, 0, 0), 2)
                        # cv2.line(color_frame, point_B, point_D, (0, 0, 0), 2)
                        """

                        """
                        AC_line = np.linalg.norm(np.array(point_A) - np.array(point_C))
                        BD_line = np.linalg.norm(np.array(point_B) - np.array(point_D))

                        if AC_line > BD_line:
                            cv2.line(color_frame, point_A, point_C, (0, 0, 0), 2)
                        else:
                            cv2.line(color_frame, point_B, point_D, (0, 0, 0), 2)
                        """

                # Tìm pixel khác 0
                pixel_cnt = cv2.countNonZero(binary)

                # Hiển thị ảnh kết quả
                cv2.imshow('Color frame', color_frame)
                # cv2.imshow('Background frame', background_gray)
                # cv2.imshow('Difference frame', frame)
                # cv2.imshow('Gray frame', gray)
                cv2.imshow('Binary frame', binary)

                key = cv2.waitKey(1)
                if key == 27:  # ESC button
                    break
