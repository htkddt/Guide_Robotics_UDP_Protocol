import sys
import os
import numpy as np

# Old : Int_matrix = np.array([[608.32574463, 0, 330.66174316], [0, 606.69873047, 242.11862183], [0, 0, 1]])
# New : Int_matrix = np.array([[608.33, 0, 330.66], [0, 606.7, 242.12], [0, 0, 1]])

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
