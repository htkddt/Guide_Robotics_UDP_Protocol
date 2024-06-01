import sys
import os
import numpy as np

Position_folder_path = 'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Test'
Position_files = [f for f in os.listdir(Position_folder_path) if f.endswith('.txt')]

Int_matrix = np.array([[608.33, 0, 330.66], [0, 606.7, 242.12], [0, 0, 1]])

index_folder = 1


def read_matrix():
    txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Cam2base\\Value\\Data_({index_folder})\\Result_cam2base_value.txt'

    with open(txt, 'r') as file:
        line = file.readlines()

        if len(line) == 12:
            R00, R01, R02, R10, R11, R12, R20, R21, R22, t00, t10, t20 = map(float, line)

            R = np.array([[R00, R01, R02],
                          [R10, R11, R12],
                          [R20, R21, R22]])

            t = np.array([[t00],
                          [t10],
                          [t20]])

    return R, t


def read_position(index):
    txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Test\\position_({index}).txt'

    with open(txt, 'r') as file:
        line = file.readlines()

        if len(line) == 17:
            u, v, Xc, Yc, Zc, Xw, Yw, Zw, _, _, _, _, _, _, _, _, _ = map(float, line)

            Pixel = np.array([[u],
                              [v],
                              [1.0]])

            # Tọa độ 3D của camera phải là ZYX thay vì XYZ
            Wc = np.array([[Xc],
                           [Yc],
                           [Zc],
                           [1.0]])

            Wr = np.array([[Xw],
                           [Yw],
                           [Zw],
                           [1.0]])

    return Pixel, Wc, Wr


R_cam2base, t_cam2base = read_matrix()
T_cam2base = np.vstack(((np.hstack((R_cam2base, t_cam2base))), ([0, 0, 0, 1])))

print("T_cam2base = \n" + str(T_cam2base) + '\n')

for i in range(1, 10):
    P_matrix, W_camera, W_robot = read_position(i)
    W_gripper = T_cam2base.dot(W_camera)

    print("index = " + str(i) + '\n' +
          "W_camera = \n" + str(W_camera) + '\n' +
          "W_robot = \n" + str(W_robot) + '\n' +
          "W_gripper = \n" + str(W_gripper) + '\n')
