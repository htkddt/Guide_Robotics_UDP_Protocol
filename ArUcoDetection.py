# Huỳnh Tuấn Kiệt - 2010364
import sys
import cv2
import numpy as np


def ARUCO_Detection(frame, Int_matrix, detector, markerLength):
    rve = None
    tve = None
    obj = None
    u = None
    v = None

    corners, ids, rejected = detector.detectMarkers(frame, rejectedImgPoints=[])
    # Hàm detect Aruco trả về corners là 1 mảng chứa các cặp giá trị là tọa độ (x,y) của 4 góc Aruco
    # Mà tọa độ này cũng chính là điểm ảnh 2D mà camera nhìn thấy

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 0, 255))
            cv2.aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 255, 0))

            c = corners[i][0]
            # Tọa độ tâm hay điểm ảnh tại tâm được tính bằng trung bình cộng tọa độ của 4 điểm góc
            u, v = np.mean(c[:, 0]), np.mean(c[:, 1])

            rve, tve, obj = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, Int_matrix, None)

            axis_length = 32.5

            for r, t in zip(rve, tve):
                # Tính toán các điểm của hệ trục tọa độ
                points = np.float32([[0, 0, 0],
                                     [axis_length, 0, 0],
                                     [0, axis_length, 0],
                                     [0, 0, axis_length]]).reshape(-1, 3)
                img_points, _ = cv2.projectPoints(points, r, t, Int_matrix, None)

                img_points = np.int32(img_points)

                # Vẽ các đoạn thẳng kết hợp với điểm bắt đầu
                frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[1].ravel()),
                                 (255, 0, 0),
                                 3)  # X-axis (red)
                frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[2].ravel()),
                                 (0, 255, 0),
                                 3)  # Y-axis (green)
                frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[3].ravel()),
                                 (0, 0, 255),
                                 3)  # Z-axis (blue)

    return frame, rve, tve, obj, u, v
