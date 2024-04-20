import sys
import os
import cv2
import numpy as np
# Test source

Int_matrix = np.array([[608.32574463, 0, 330.66174316],
                       [0, 606.69873047, 242.11862183],
                       [0, 0, 1]])

position_folder_path = 'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Data_(2)'
position_files = [f for f in os.listdir(position_folder_path) if f.endswith('.txt')]

image_folder_path = 'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Data_(2)'
image_files = [f for f in os.listdir(image_folder_path) if f.lower().endswith('.jpg')]

object_list = []
image_list = []

grid_size = (5, 5)  # Kích thước của lưới (số ô theo chiều ngang, chiều dọc)
grid_points = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
grid_points[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)

for index in range(1, len(image_files) + 1):
    image = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Data_(2)\\image_({index}).jpg'
    print("Path: " + str(image) + '\n')
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, grid_size, None)

    print("Status " + str(ret) + '\n')

    if ret:
        image_list.append(corners)
        object_list.append(grid_points)

# image_size = gray.shape[::-1]  # Kích thước ảnh (width, height)
image_size = (640, 480)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_list, image_list, image_size, None, None)

# In ra ma trận intrinsic và distortion coefficients
print("Ma trận intrinsic (camera matrix):")
print(mtx)
print("\nDistortion coefficients:")
print(dist)
