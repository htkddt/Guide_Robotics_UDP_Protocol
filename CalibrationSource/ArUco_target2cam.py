import os
from CameraSource.RealsenseCamera import *
# R_target2cam (3x3): Ma trận xoay giữa Aruco và Camera (Opencv)
# t_target2cam (3x1): Ma trận dịch chuyển giữa Aruco và Camera tại tâm của Aruco (Opencv)

index_folder = 1

Int_matrix = np.array([[608.33, 0, 330.66], [0, 606.7, 242.12], [0, 0, 1]])

image_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Data_({index_folder})'
image_files = [f for f in os.listdir(image_folder_path) if f.lower().endswith('.jpg')]

position_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Data_({index_folder})'
position_files = [f for f in os.listdir(position_folder_path) if f.endswith('.txt')]

# Khởi tạo ArUco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# Khởi tạo ArUco parameters
arucoParams = cv2.aruco.DetectorParameters()

# Khởi tạo đối tượng ArUcoDetector
# detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

result_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Target2cam\\Data_({index_folder})'
value_folder_path = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Value\\Target2cam\\Data_({index_folder})'

for index in range(1, len(image_files) + 1):
    image = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Data_({index_folder})\\image_({index}).jpg'
    txt = f'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Position\\Data_({index_folder})\\position_({index}).txt'

    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    # cv2.aruco.drawDetectedMarkers(img, corners, ids, (0, 0, 255))
    # cv2.aruco.drawDetectedMarkers(img, corners, borderColor=(0, 255, 0))

    markerLength = 65  # 65 mm
    rve, tve, obj = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, Int_matrix, None)

    """
    axis_length = 65

    for r, t in zip(rve, tve):
        # Tính toán các điểm của hệ trục tọa độ
        points = np.float32([[0, 0, 0],
                             [axis_length, 0, 0],
                             [0, axis_length, 0],
                             [0, 0, axis_length]]).reshape(-1, 3)
        # print(str(points) + '\n')

        img_points, _ = cv2.projectPoints(points, r, t, Int_matrix, None)

        img_points = np.int32(img_points)

        # Vẽ các đoạn thẳng kết hợp với điểm bắt đầu
        img = cv2.line(img, tuple(img_points[0].ravel()), tuple(img_points[1].ravel()), (0, 0, 255), 3)
        # X-axis (red)
        img = cv2.line(img, tuple(img_points[0].ravel()), tuple(img_points[2].ravel()), (0, 255, 0), 3)
        # Y-axis (green)
        img = cv2.line(img, tuple(img_points[0].ravel()), tuple(img_points[3].ravel()), (255, 0, 0), 3)
        # Z-axis (blue)
    """

    # cv2.imshow("Aruco detect", img)
    # cv2.waitKey(1000)

    with open(txt, 'r') as file:
        line = file.readlines()

        if len(line) == 17:
            _, _, Xc, Yc, Zc, _, _, _, _, _, _, _, _, _, _, _, _ = map(float, line)

    R_target2cam, _ = cv2.Rodrigues(rve)
    t_target2cam = np.array([[Xc],
                             [Yc],
                             [Zc]])

    print("R_t2c(" + str(index) + ") = \n" + str(R_target2cam) + '\n' +
          "t_t2c(" + str(index) + ") = \n" + str(t_target2cam) + '\n' +
          "rvec(" + str(index) + ") = \n" + str(rve) + '\n' +
          "tvec(" + str(index) + ") = \n" + str(tve))

    if not os.path.exists(result_folder_path):
        os.makedirs(result_folder_path)
    result_file_name = f'target2cam_({index}).txt'
    result_file_path = os.path.join(result_folder_path, result_file_name)
    with open(result_file_path, 'w') as result_file:
        result_file.write("rvec = \n" + str(rve) + '\n\n' +
                          "tvec = \n" + str(tve) + '\n\n' +
                          "R_target2cam = \n" + str(R_target2cam) + '\n\n' +
                          "t_target2cam = \n" + str(t_target2cam))

    if not os.path.exists(value_folder_path):
        os.makedirs(value_folder_path)
    value_file_name = f'target2cam_({index}).txt'
    value_file_path = os.path.join(value_folder_path, value_file_name)
    with open(value_file_path, 'w') as value_file:
        value_file.write(str(R_target2cam[0][0]) + '\n' +
                         str(R_target2cam[0][1]) + '\n' +
                         str(R_target2cam[0][2]) + '\n' +
                         str(R_target2cam[1][0]) + '\n' +
                         str(R_target2cam[1][1]) + '\n' +
                         str(R_target2cam[1][2]) + '\n' +
                         str(R_target2cam[2][0]) + '\n' +
                         str(R_target2cam[2][1]) + '\n' +
                         str(R_target2cam[2][2]) + '\n' +
                         str(t_target2cam[0][0]) + '\n' +
                         str(t_target2cam[1][0]) + '\n' +
                         str(t_target2cam[2][0]))
