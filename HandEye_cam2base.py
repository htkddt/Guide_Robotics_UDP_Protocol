# Huỳnh Tuấn Kiệt - 2010364
import sys
import os
import cv2
import numpy as np
# R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)

# Eye-in-hand:
# R_gripper2base = R_gripper2base
# t_gripper2base = t_gripper2base

# Eye-to-hand:
# R_gripper2base = R_base2gripper hay (R_gripper2base.T) [Chuyển vị]
# t_gripper2base = -R_gripper2base.T @ t_gripper2base [Công thức]

index_folder = 1

gripper2base_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Value\\Gripper2base\\Data_({index_folder})'
gripper2base_files = [f for f in os.listdir(gripper2base_folder_path) if f.endswith('.txt')]

target2cam_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Value\\Target2cam\\Data_({index_folder})'
target2cam_files = [f for f in os.listdir(target2cam_folder_path) if f.endswith('.txt')]


def gripper2base_read():
    R_list = []
    t_list = []
    for index in range(1, len(gripper2base_files) + 1):
        txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Value\\Gripper2base\\Data_({index_folder})\\gripper2base_({index}).txt'

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

                R_list.append(R)
                t_list.append(t)

    return R_list, t_list


def target2cam_read():
    R_list = []
    t_list = []
    for index in range(1, len(target2cam_files) + 1):
        txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Value\\Target2cam\\Data_({index_folder})\\target2cam_({index}).txt'

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

                R_list.append(R)
                t_list.append(t)

    return R_list, t_list


method = cv2.CALIB_HAND_EYE_PARK

R_gripper2base, t_gripper2base = gripper2base_read()
R_target2cam, t_target2cam = target2cam_read()

R_cam2base, t_cam2base = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)

print("R_cam2base = \n" + str(R_cam2base) + '\n' + "t_cam2base = \n" + str(t_cam2base))

matrix_save_path_txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Cam2base\\Matrix\\Data_({index_folder})'
value_save_path_txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Cam2base\\Value\\Data_({index_folder})'

if not os.path.exists(matrix_save_path_txt):
    os.makedirs(matrix_save_path_txt)

txt_file_name_c2b_matrix = 'Result_cam2base_matrix.txt'
txt_file_name_c2b_value = 'Result_cam2base_value.txt'
txt_file_path_c2b_matrix = os.path.join(matrix_save_path_txt, txt_file_name_c2b_matrix)
txt_file_path_c2b_value = os.path.join(value_save_path_txt, txt_file_name_c2b_value)

with open(txt_file_path_c2b_matrix, 'w') as txt_file:
    txt_file.write("R_cam2base = \n" + str(R_cam2base) + '\n' +
                   "t_cam2base = \n" + str(t_cam2base))

with open(txt_file_path_c2b_value, 'w') as txt_file:
    txt_file.write(str(R_cam2base[0][0]) + '\n' +
                   str(R_cam2base[0][1]) + '\n' +
                   str(R_cam2base[0][2]) + '\n' +
                   str(R_cam2base[1][0]) + '\n' +
                   str(R_cam2base[1][1]) + '\n' +
                   str(R_cam2base[1][2]) + '\n' +
                   str(R_cam2base[2][0]) + '\n' +
                   str(R_cam2base[2][1]) + '\n' +
                   str(R_cam2base[2][2]) + '\n' +
                   str(t_cam2base[0][0]) + '\n' +
                   str(t_cam2base[1][0]) + '\n' +
                   str(t_cam2base[2][0]))



