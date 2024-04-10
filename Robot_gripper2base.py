import sys
import os
import cv2
import scipy
import math
import numpy as np
# R_gripper2base: Ma trận xoay giữa Gripper và Base (Tính toán từ vị trí x,y,z,roll,pitch,yall của Robot)
# t_gripper2base: Ma trận dịch chuyển giữa Gripper và Base (Tính toán từ vị trí x,y,z,roll,pitch,yall của Robot)

index_folder = 1


def transform_rotation(method, Roll, Pitch, Yaw):
    rotation = None
    r_X = np.array([[1, 0, 0],
                    [0, math.cos(Roll), -math.sin(Roll)],
                    [0, math.sin(Roll), math.cos(Roll)]])

    r_Y = np.array([[math.cos(Pitch), 0, math.sin(Pitch)],
                    [0, 1, 0],
                    [-math.sin(Pitch), 0, math.cos(Pitch)]])

    r_Z = np.array([[math.cos(Yaw), -math.sin(Yaw), 0],
                    [math.sin(Yaw), math.cos(Yaw), 0],
                    [0, 0, 1]])

    if method == "XYZ":
        rotation = r_Z @ r_Y @ r_X
    elif method == "ZYX":
        rotation = r_X @ r_Y @ r_Z
    elif method == "ZXY":
        rotation = r_Y @ r_X @ r_Z

    return rotation


position_folder_path = f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Data_({index_folder})'
position_files = [f for f in os.listdir(position_folder_path) if f.endswith('.txt')]

result_folder_path = f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Gripper2base\\Data_({index_folder})'
value_folder_path = f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Value\\Gripper2base\\Data_({index_folder})'

for index in range(1, len(position_files) + 1):
    txt = f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Data_({index_folder})\\position_({index}).txt'

    with open(txt, 'r') as file:
        line = file.readlines()

        if len(line) == 17:
            _, _, _, _, _, Xw, Yw, Zw, Rx, Ry, Rz, S, L, U, R, B, T = map(float, line)

            """
            print("txt: " + txt + '\n' + "index = " + str(index) + '\n' +
                  str(Xw) + '\n' + str(Yw) + '\n' + str(Zw) + '\n' +
                  str(Rx) + '\n' + str(Ry) + '\n' + str(Rz) + '\n\n')
            """

            """
            R_, t_ = forward((S * math.pi) / 180,
                             (L * math.pi) / 180,
                             (U * math.pi) / 180,
                             (R * math.pi) / 180,
                             (B * math.pi) / 180,
                             (T * math.pi) / 180)
            """

            # Thay đổi tọa độ Robot theo giá trị tham chiếu của ArUco
            t_g2b = np.array([[Xw],
                              [Yw],
                              [Zw]])

            R_g2b = transform_rotation("XYZ", math.radians(Rx), math.radians(Ry), math.radians(Rz))

            print("R_g2b(" + str(index) + ") = \n" + str(R_g2b) + '\n' +
                  "t_g2b(" + str(index) + ") = \n" + str(t_g2b))

            R_base2gripper = R_g2b.T
            t_base2gripper = -R_base2gripper @ t_g2b

            """
            T_g2b = np.vstack(((np.hstack((R_scipy, t_g2b))), ([0, 0, 0, 1.0])))
            T_b2g = np.linalg.inv(T_g2b)

            R_base2gripper = np.array([[T_b2g[0][0], T_b2g[0][1], T_b2g[0][2]],
                                       [T_b2g[1][0], T_b2g[1][1], T_b2g[1][2]],
                                       [T_b2g[2][0], T_b2g[2][1], T_b2g[2][2]]])

            t_base2gripper = np.array([[T_b2g[0][3]],
                                       [T_b2g[1][3]],
                                       [T_b2g[2][3]]])
            """

            """
            print("R_base2gripper(" + str(index) + ") = \n" + str(R_base2gripper) + '\n' +
                  "R_b2g(" + str(index) + ") = \n" + str(R_b2g) + '\n' +
                  "t_base2gripper(" + str(index) + ") = \n" + str(t_base2gripper) + '\n' +
                  "t_b2g(" + str(index) + ") = \n" + str(t_b2g) + '\n\n')
            """

            if not os.path.exists(result_folder_path):
                os.makedirs(result_folder_path)
            result_file_name = f'gripper2base_({index}).txt'
            result_file_path = os.path.join(result_folder_path, result_file_name)
            with open(result_file_path, 'w') as result_file:
                result_file.write("R_gripper2base = \n" + str(R_base2gripper) + '\n\n' +
                                  "t_gripper2base = \n" + str(t_base2gripper))

            if not os.path.exists(value_folder_path):
                os.makedirs(value_folder_path)
            value_file_name = f'gripper2base_({index}).txt'
            value_file_path = os.path.join(value_folder_path, value_file_name)
            with open(value_file_path, 'w') as value_file:
                value_file.write(str(R_base2gripper[0][0]) + '\n' +
                                 str(R_base2gripper[0][1]) + '\n' +
                                 str(R_base2gripper[0][2]) + '\n' +
                                 str(R_base2gripper[1][0]) + '\n' +
                                 str(R_base2gripper[1][1]) + '\n' +
                                 str(R_base2gripper[1][2]) + '\n' +
                                 str(R_base2gripper[2][0]) + '\n' +
                                 str(R_base2gripper[2][1]) + '\n' +
                                 str(R_base2gripper[2][2]) + '\n' +
                                 str(t_base2gripper[0][0]) + '\n' +
                                 str(t_base2gripper[1][0]) + '\n' +
                                 str(t_base2gripper[2][0]))