import math
import os
import sys
import cv2
import struct
import socket
import time
import scipy
import serial
import numpy as np

import Cam2base
import ArUcoDetection
import ColorDetection

from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication, QMessageBox

from scipy.spatial.transform import Rotation
from pymodbus.client import ModbusTcpClient
from RealsenseCamera import *
from YoloDetection import *
from YoloSegmentation import *
from Guide_htkddt import Ui_MainWindow

"""
# Khai báo MainWindow() loại 1
class MainWindow():
    def __int__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)

    def show(self):
        self.main_win.show()
"""


# Note:
# Check Mode control: Auto Button and Manual Button
# Check style button of set_button_style function
# Coding classify and update position in Job Robot


class MainWindow(QMainWindow):
    # Khai báo MainWindow() loại 2

    def __init__(self):
        super().__init__()

        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)

        self.index_data = 0
        self.index_folder = 4

        self.uic.lb_Gripper.setText("Gripper: None")
        self.uic.txt_dt.setText("0")

        self.Int_matrix = None
        self.T_cam2base = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.X_ = None
        self.Y_ = None
        self.Z_ = None
        self.Roll_ = None
        self.Pitch_ = None
        self.Yaw_ = None

        # self.Mode_Control = self.uic.cBox_Mode.currentText()
        # print("Mode control: " + self.Mode_Control)

        self.flag_Auto = None
        self.flag_Manual = None

        self.flag_Capture_ArUco = None
        self.flag_Capture_Object = None
        self.flag_Capture_Background = None
        self.flag_RGB = None
        self.flag_Binary = None
        self.flag_Depth = None
        self.flag_GetPoint = None
        self.flag_Enable_Job = None
        self.flag_Data_Position = None
        self.flag_Process = None
        self.flag_Select = None

        self.flag_Detect_ArUco = None
        self.flag_Detect_Color = None
        self.flag_Detect_YoLo = None

        self.flag_Conveyor = None

        # Khởi tạo ArUco dictionary
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        # Khởi tạo ArUco parameters
        self.arucoParams = cv2.aruco.DetectorParameters()
        # Khởi tạo đối tượng ArUcoDetector
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.socket = RobotSocket()
        # self.conveyor = ConveyorSocket()

        self.serial = SerialProcess()
        self.serial.message.connect(self.update_serial)
        self.method = 0

        self.camera = CameraThread()
        self.camera.image.connect(self.update_frame)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_timer)

        self.uic.slider_Dis_mm.valueChanged.connect(self.update_txt1_Dis)
        self.uic.slider_Dis_deg.valueChanged.connect(self.update_txt2_Dis)
        self.uic.slider_Spe_mm.valueChanged.connect(self.update_txt1_Spe)
        self.uic.slider_Spe_deg.valueChanged.connect(self.update_txt2_Spe)

        # self.uic.cBox_Mode.currentIndexChanged.connect(self.update_mode)
        self.uic.btn_Auto.clicked.connect(self.mode_auto)
        self.uic.btn_Manual.clicked.connect(self.mode_manual)
        self.uic.btn_Robot_ConDis.clicked.connect(self.con_dis_robot_action)
        self.uic.btn_Serial_ConDis.clicked.connect(self.con_dis_serial_action)
        self.uic.btn_Open_Close.clicked.connect(self.open_close_action)

        self.uic.btn_Servo.clicked.connect(self.servo_action)
        self.uic.btn_Home.clicked.connect(self.go_home_action)

        self.uic.btn_Get_Position.clicked.connect(self.get_position_action)

        # self.uic.btn_Conveyor.clicked.connect(self.conveyor_action)

        self.uic.btn_Serial.clicked.connect(self.update_txt_Pulse)

        self.uic.btn_Start.clicked.connect(self.start_time)
        self.run_time = 0
        self.uic.btn_Stop.clicked.connect(self.stop_time)
        self.end_time = 0

        self.uic.btn_X_Inc.clicked.connect(self.X_Inc_action)
        self.uic.btn_X_Dec.clicked.connect(self.X_Dec_action)

        self.uic.btn_Y_Inc.clicked.connect(self.Y_Inc_action)
        self.uic.btn_Y_Dec.clicked.connect(self.Y_Dec_action)

        self.uic.btn_Z_Inc.clicked.connect(self.Z_Inc_action)
        self.uic.btn_Z_Dec.clicked.connect(self.Z_Dec_action)

        self.uic.btn_Roll_Inc.clicked.connect(self.Roll_Inc_action)
        self.uic.btn_Roll_Dec.clicked.connect(self.Roll_Dec_action)

        self.uic.btn_Pitch_Inc.clicked.connect(self.Pitch_Inc_action)
        self.uic.btn_Pitch_Dec.clicked.connect(self.Pitch_Dec_action)

        self.uic.btn_Yaw_Inc.clicked.connect(self.Yaw_Inc_action)
        self.uic.btn_Yaw_Dec.clicked.connect(self.Yaw_Dec_action)

        self.uic.btn_Move_Job.clicked.connect(self.move_job_action)
        self.uic.btn_Exit_Job.clicked.connect(self.exit_job_action)
        self.uic.btn_Pos_pick.clicked.connect(self.pos_pick_action)
        self.uic.btn_Move_test.clicked.connect(self.move_test_action)

        self.uic.btn_Capture_Aruco.clicked.connect(self.capture_aruco_action)
        self.uic.btn_Capture_Object.clicked.connect(self.capture_object_action)
        self.uic.btn_Background.clicked.connect(self.capture_background_action)

        self.uic.btn_Aruco_Detect.clicked.connect(self.aruco_detect_action)
        self.uic.btn_COLOR_Detect.clicked.connect(self.COLOR_detect_action)
        self.uic.btn_YOLO_detect.clicked.connect(self.YOLO_detect_action)

        self.uic.btn_RGB.clicked.connect(self.display_RGB_frame)
        self.uic.btn_Binary.clicked.connect(self.display_Binary_frame)
        self.uic.btn_Depth.clicked.connect(self.display_Depth_frame)

    def close(self):
        self.camera.running = False
        self.flag_RGB = False
        self.flag_Binary = False
        self.flag_Depth = False
        self.camera.quit()

    def frame_data_send(self, Data_Part, Request_ID, Command, Instance, Attribute, Service, Data):
        Identifier = bytes([0x59, 0x45, 0x52, 0x43])  # YERC
        Header = bytes([0x20, 0x00])
        Reserve_1 = bytes([0x03])
        Processing_div = bytes([0x01])
        ACK = bytes([0x00])
        Block_No = bytes([0x00, 0x00, 0x00, 0x00])
        Reserve_2 = bytes([0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39])
        Padding = bytes([0x00, 0x00])
        Data_Part_Length = int.from_bytes(Data_Part, byteorder='little', signed=True)

        format_frame = struct.Struct(f'!4s 2s 2s 1s 1s 1s 1s 4s 8s 2s 2s 1s 1s 2s {Data_Part_Length}s')

        frame_data = format_frame.pack(Identifier, Header, Data_Part, Reserve_1, Processing_div, ACK, Request_ID,
                                       Block_No, Reserve_2, Command, Instance, Attribute, Service, Padding, Data)

        return frame_data

    def set_button_style(self):
        self.uic.btn_Robot_ConDis.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: black;
                font-size: 20px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #1abc9c;
            }
        """)

        self.uic.btn_Serial_ConDis.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: black;
                font-size: 20px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #1abc9c;
            }
        """)

        self.uic.btn_YOLO_detect.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: black;
                font-size: 20px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #1abc9c;
            }
        """)

        self.uic.btn_COLOR_Detect.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: black;
                font-size: 20px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #1abc9c;
            }
        """)

        self.uic.btn_Servo.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: black;
                font-size: 20px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #1abc9c;
            }
        """)

    def con_dis_robot_action(self):
        if self.uic.btn_Connect_Disconnect.text() == "Connect":
            self.uic.btn_Connect_Disconnect.setText("Disconnect")

            # Lấy dữ liệu IP và Port
            IP = self.uic.txt_IP.text()
            Port = int(self.uic.txt_Port.text())

            self.socket.robot_address(IP, Port)
            self.socket.status = 0
            self.socket.start()
            # self.conveyor.start()
            self.uic.lb_Connect_Disconnect.setText("Connected")

        elif self.uic.btn_Connect_Disconnect.text() == "Disconnect":
            self.uic.btn_Connect_Disconnect.setText("Connect")
            self.socket.disconnect()
            # self.conveyor.disconnect()
            self.uic.lb_Connect_Disconnect.setText("Disconnected")

    def con_dis_serial_action(self):
        if self.uic.btn_Serial_ConDis.text() == "Connect":
            self.uic.btn_Serial_ConDis.setText("Disconnect")
            Port = self.uic.txt_Port_.text()
            Baud = int(self.uic.txt_Baudrate.text())
            self.serial.serial_connect(Port)
            self.serial.start()
            self.uic.lb_Connect_Disconnect_.setText("Connected")

        elif self.uic.btn_Serial_ConDis.text() == "Disconnect":
            self.uic.btn_Serial_ConDis.setText("Connect")
            self.serial.serial_disconnect()
            self.uic.lb_Connect_Disconnect_.setText("Disconnected")

    def open_close_action(self):
        self.camera.flag_Detect_YOLO = False
        self.uic.btn_YOLO_detect.setText("YOLO Detect ON")
        self.camera.flag_Detect_COLOR = False
        self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")

        if self.uic.btn_Open_Close.text() == "Open Camera":
            self.uic.btn_Open_Close.setText("Close Camera")
            if not self.camera.isRunning():
                self.flag_RGB = True
                self.camera.cnt = 0
                self.camera.running = True
                self.camera.start()
                R, t = Cam2base.read_matrix()
                self.T_cam2base = np.vstack(((np.hstack((R, t))), ([0, 0, 0, 1.0])))
                self.Int_matrix, Dist = self.camera.matrix()
                self.cx = int(self.Int_matrix[0][2])
                self.cy = int(self.Int_matrix[1][2])
                self.fx = self.Int_matrix[0][0]
                self.fy = self.Int_matrix[1][1]
                print("Intrinsics matrix = \n" + str(self.Int_matrix) + '\n' +
                      "Extrinsic matrix = \n" + str(self.T_cam2base) + '\n' +
                      "Distortion coefficients = " + str(Dist) + '\n' +
                      "cx = " + str(self.cx) + '\n' +
                      "cy = " + str(self.cy) + '\n' +
                      "fx = " + str(self.fx) + '\n' +
                      "fy = " + str(self.fy))

        elif self.uic.btn_Open_Close.text() == "Close Camera":
            self.uic.btn_Open_Close.setText("Open Camera")
            if self.camera.isRunning():
                self.camera.running = False
                self.flag_RGB = False
                self.flag_Binary = False
                self.flag_Depth = False
                self.camera.quit()

    def mode_auto(self):
        self.flag_Auto = True
        self.flag_Manual = False

    def mode_manual(self):
        self.flag_Auto = False
        self.flag_Manual = True

    def update_txt1_Dis(self):
        value = self.uic.slider_Dis_mm.value()
        self.uic.txt1_Dis_Value.setText(str(value))

    def update_txt2_Dis(self):
        value = self.uic.slider_Dis_deg.value()
        self.uic.txt2_Dis_Value.setText(str(value))

    def update_txt1_Spe(self):
        value = self.uic.slider_Spe_mm.value()
        self.uic.txt1_Spe_Value.setText(str(value))

    def update_txt2_Spe(self):
        value = self.uic.slider_Spe_deg.value()
        self.uic.txt2_Spe_Value.setText(str(value))

    def update_txt_Pulse(self):
        Value = self.uic.txt_Pulse.text()
        self.serial_process_action(Value)

    def display_RGB_frame(self):
        self.flag_RGB = True
        self.flag_Binary = False
        self.flag_Depth = False

    def display_Binary_frame(self):
        self.flag_RGB = False
        self.flag_Binary = True
        self.flag_Depth = False

    def display_Depth_frame(self):
        self.flag_RGB = False
        self.flag_Binary = False
        self.flag_Depth = True

    def capture_aruco_action(self):
        self.flag_Capture_ArUco = True

    def capture_object_action(self):
        self.flag_Capture_Object = True

    def capture_background_action(self):
        self.flag_Capture_Background = True

    def aruco_detect_action(self):
        if self.uic.btn_Aruco_Detect.text() == "ArUco Detect ON":
            self.flag_Detect_ArUco = True
            self.camera.flag_Detect_COLOR = False
            self.camera.flag_Detect_YOLO = False
            self.uic.btn_Aruco_Detect.setText("ArUco Detect OFF")
            self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")
            self.uic.btn_YOLO_detect.setText("YOLO Detect ON")

        elif self.uic.btn_Aruco_Detect.text() == "ArUco Detect OFF":
            self.flag_Detect_ArUco = False
            self.camera.flag_Detect_COLOR = False
            self.camera.flag_Detect_YOLO = False
            self.uic.btn_Aruco_Detect.setText("ArUco Detect ON")
            self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")
            self.uic.btn_YOLO_detect.setText("YOLO Detect ON")

    def COLOR_detect_action(self):
        if self.uic.btn_COLOR_Detect.text() == "COLOR Detect ON":
            self.camera.flag_Detect_COLOR = True
            self.flag_Detect_ArUco = False
            self.camera.flag_Detect_YOLO = False
            self.uic.btn_COLOR_Detect.setText("COLOR Detect OFF")
            self.uic.btn_Aruco_Detect.setText("ArUco Detect ON")
            self.uic.btn_YOLO_detect.setText("YOLO Detect ON")

        elif self.uic.btn_COLOR_Detect.text() == "COLOR Detect OFF":
            self.camera.flag_Detect_COLOR = False
            self.flag_Detect_ArUco = False
            self.camera.flag_Detect_YOLO = False
            self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")
            self.uic.btn_Aruco_Detect.setText("ArUco Detect ON")
            self.uic.btn_YOLO_detect.setText("YOLO Detect ON")

    def YOLO_detect_action(self):
        if self.uic.btn_YOLO_detect.text() == "YOLO Detect ON":
            self.camera.flag_Detect_YOLO = True
            self.flag_Detect_ArUco = False
            self.camera.flag_Detect_COLOR = False
            self.uic.btn_YOLO_detect.setText("YOLO Detect OFF")
            self.uic.btn_Aruco_Detect.setText("ArUco Detect ON")
            self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")

        elif self.uic.btn_YOLO_detect.text() == "YOLO Detect OFF":
            self.camera.flag_Detect_YOLO = False
            self.flag_Detect_ArUco = False
            self.camera.flag_Detect_COLOR = False
            self.uic.btn_YOLO_detect.setText("YOLO Detect ON")
            self.uic.btn_Aruco_Detect.setText("ArUco Detect ON")
            self.uic.btn_COLOR_Detect.setText("COLOR Detect ON")

    def start_time(self):
        self.run_time = time.time()

    def stop_time(self):
        self.end_time = time.time()
        self.uic.txt_dt.setText(str(round((self.end_time - self.run_time), 2)))

    def axis_config_function(self):
        Data_Part = bytes([0x04, 0x00])
        Request_ID = bytes([0x00])
        Command = bytes([0x74, 0x00])
        Attribute = bytes([0x00])
        Service = bytes([0x01])
        Data = bytes([0x00, 0x00, 0x00, 0x00])
        # Instance = None
        # Khi config trục Data trong Request là không có (0x00 0x00 0x00 0x00)
        # Khi config trục kiểu Pulse thì Instance = 1 (0x01 0x00)
        # Khi config trục kiểu Cartesian thì Instance = 101 (0x65 0x00)

        status = 0

        while status < 2:
            if status == 0:
                Instance = bytes([0x01, 0x00])

                frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

                self.socket.send_function(frame)
                self.socket.received_function()

            elif status == 1:
                Instance = bytes([0x65, 0x00])

                frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

                self.socket.send_function(frame)
                self.socket.received_function()

            status = status + 1

        return True

    def servo_action(self):
        Data = None
        Data_Part = bytes([0x04, 0x00])
        Request_ID = bytes([0x01])
        Command = bytes([0x83, 0x00])
        Instance = bytes([0x02, 0x00])
        Attribute = bytes([0x01])
        Service = bytes([0x10])

        if self.uic.btn_Servo.text() == "Servo On":
            Data = bytes([0x01, 0x00, 0x00, 0x00])
            frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)
            self.socket.send_function(frame)
            self.socket.received_function()

            if self.socket.flag_connected:
                status = self.axis_config_function()
                if status:
                    self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 1)
                    self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 2)
                    self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 3)
                    self.set_position_action(-54000, -286303, 27000, 1800000, 0, -900000, 4)
                    self.set_byte_action(0, 1)
                    self.set_byte_action(0, 2)
                    self.set_byte_action(0, 3)
                    self.set_byte_action(0, 4)
                    self.set_byte_action(1, 0)
                    self.serial_process_action(80)
                    self.flag_Select = True
                    self.timer.start(100)
            self.uic.btn_Servo.setText("Servo Off")

        elif self.uic.btn_Servo.text() == "Servo Off":
            Data = bytes([0x02, 0x00, 0x00, 0x00])
            frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)
            self.socket.send_function(frame)
            self.socket.received_function()
            self.set_byte_action(0, 0)
            self.timer.stop()
            self.uic.btn_Servo.setText("Servo On")

        self.get_position_action()

    def move_command_action(self, Speed_type, X, Y, Z, Rx, Ry, Rz):
        # Speed type = 1 (0x01) là vận tốc dịch chuyển
        # Speed type = 2 (0x02) là vận tốc xoay
        Speed_value = None

        if Speed_type == 0:
            Speed_type = 1
            Speed_value = 2000
        elif Speed_type == 1:
            Speed_value = self.uic.slider_Spe_mm.value()
        elif Speed_type == 2:
            Speed_value = self.uic.slider_Spe_deg.value()

        Data_Part = bytes([0x68, 0x00])
        Request_ID = bytes([0x02])
        Command = bytes([0x8A, 0x00])
        Instance = bytes([0x02, 0x00])
        Attribute = bytes([0x01])
        Service = bytes([0x02])

        Data_Control_Group = bytes([0x01, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00])

        Data_Speed_type = Speed_type.to_bytes(4, byteorder='little', signed=False)
        Data_Speed_value = Speed_value.to_bytes(4, byteorder='little', signed=False)
        Data_Distance_type = bytes([0x11, 0x00, 0x00, 0x00])
        Data_X = X.to_bytes(4, byteorder='little', signed=True)
        Data_Y = Y.to_bytes(4, byteorder='little', signed=True)
        Data_Z = Z.to_bytes(4, byteorder='little', signed=True)
        Data_Roll = Rx.to_bytes(4, byteorder='little', signed=True)
        Data_Pitch = Ry.to_bytes(4, byteorder='little', signed=True)
        Data_Yaw = Rz.to_bytes(4, byteorder='little', signed=True)
        Data_Reservation_1 = bytes([0x00, 0x00, 0x00, 0x00])
        Data_Reservation_2 = bytes([0x00, 0x00, 0x00, 0x00])
        Data_Type = bytes([0x00, 0x00, 0x00, 0x00])
        Data_Extended = bytes([0x00, 0x00, 0x00, 0x00])
        Data_Tool = bytes([0x00, 0x00, 0x00, 0x00])
        Data_Define = bytes([0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00])

        Data = []

        Data.extend(Data_Control_Group)
        Data.extend(Data_Speed_type)
        Data.extend(Data_Speed_value)
        Data.extend(Data_Distance_type)
        Data.extend(Data_X)
        Data.extend(Data_Y)
        Data.extend(Data_Z)
        Data.extend(Data_Roll)
        Data.extend(Data_Pitch)
        Data.extend(Data_Yaw)
        Data.extend(Data_Reservation_1)
        Data.extend(Data_Reservation_2)
        Data.extend(Data_Type)
        Data.extend(Data_Extended)
        Data.extend(Data_Tool)
        Data.extend(Data_Define)

        Data = bytes(Data)

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        self.socket.received_function()

    def go_home_action(self):
        X_ = 185 * 1000
        Y_ = 0
        Z_ = 35 * 1000
        Roll_ = 180 * 10000
        Pitch_ = 0
        Yaw_ = 0

        self.move_command_action(0, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def X_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000 + int(self.uic.slider_Dis_mm.value())
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def X_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000 - int(self.uic.slider_Dis_mm.value())
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Y_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000 + int(self.uic.slider_Dis_mm.value())
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Y_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000 - int(self.uic.slider_Dis_mm.value())
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Z_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000 + int(self.uic.slider_Dis_mm.value())
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Z_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000 - int(self.uic.slider_Dis_mm.value())
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(1, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Roll_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000 + int(self.uic.slider_Dis_deg.value())
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Roll_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000 - int(self.uic.slider_Dis_deg.value())
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Pitch_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000 + int(self.uic.slider_Dis_deg.value())
        Yaw_ = Yaw * 10000

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Pitch_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000 - int(self.uic.slider_Dis_deg.value())
        Yaw_ = Yaw * 10000

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Yaw_Inc_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000 + int(self.uic.slider_Dis_deg.value())

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def Yaw_Dec_action(self):
        X = float(self.uic.txt_X.text())
        Y = float(self.uic.txt_Y.text())
        Z = float(self.uic.txt_Z.text())
        Roll = float(self.uic.txt_Roll.text())
        Pitch = float(self.uic.txt_Pitch.text())
        Yaw = float(self.uic.txt_Yaw.text())

        X_ = X * 1000
        Y_ = Y * 1000
        Z_ = Z * 1000
        Roll_ = Roll * 10000
        Pitch_ = Pitch * 10000
        Yaw_ = Yaw * 10000 - int(self.uic.slider_Dis_deg.value())

        self.move_command_action(2, int(X_), int(Y_), int(Z_), int(Roll_), int(Pitch_), int(Yaw_))

    def job_select_function(self, Data_Part, Data):
        Data_Part = Data_Part.to_bytes(2, byteorder='little', signed=True)
        Request_ID = bytes([0xFF])
        Command = bytes([0x87, 0x00])
        Instance = bytes([0x01, 0x00])
        Attribute = bytes([0x01])
        Service = bytes([0x02])

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        self.socket.received_function()

        return True

    def start_job_function(self):
        Data_Part = bytes([0x04, 0x00])
        Request_ID = bytes([0xFE])
        Command = bytes([0x86, 0x00])
        Instance = bytes([0x01, 0x00])
        Attribute = bytes([0x01])
        Service = bytes([0x10])
        Data = bytes([0x01, 0x00, 0x00, 0x00])

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        self.socket.received_function()

    def move_job_action(self):
        if self.uic.btn_Move_Job.text() == "MOVE JOB":
            ret = self.job_select_function(7, b'MOVEJOB')
            if ret:
                self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 2)
                self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 3)
                self.set_byte_action(1, 1)
                self.start_job_function()
            self.flag_Enable_Job = True
            self.serial_process_action(80)
            self.uic.btn_Move_Job.setText("RUN JOB")

        elif self.uic.btn_Move_Job.text() == "RUN JOB":
            if self.flag_Manual:
                self.flag_Process = True
                self.uic.btn_Move_Job.setText("RUNNING")
            elif self.flag_Auto:
                self.uic.btn_Move_Job.setText("RUNNING")

        elif self.uic.btn_Move_Job.text() == "RUNNING":
            self.set_position_action(int(self.X_), int(self.Y_), 27000,
                                     int(self.Roll_), int(self.Pitch_), int(self.Yaw_),
                                     2)
            self.set_position_action(int(self.X_), int(self.Y_), int(self.Z_),
                                     int(self.Roll_), int(self.Pitch_), int(self.Yaw_),
                                     3)
            self.set_byte_action(1, 2)
            self.set_byte_action(0, 5)
            if self.flag_Manual:
                self.uic.btn_Move_Job.setText("RUN JOB")

    def exit_job_action(self):
        if self.uic.btn_Move_Job.text() == "RUNNING":
            self.uic.btn_Move_Job.setText("MOVE JOB")
        elif self.uic.btn_Move_Job.text() == "RUN JOB":
            self.uic.btn_Move_Job.setText("MOVE JOB")
        self.flag_Enable_Job = False
        self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 2)
        self.set_position_action(230000, 0, 27000, 1800000, 0, 0, 3)
        self.set_byte_action(1, 4)
        self.set_byte_action(0, 1)
        self.set_byte_action(0, 2)
        self.set_byte_action(0, 3)
        # self.go_home_action()

    def pos_pick_action(self):
        self.flag_Enable_Job = True
        print("X = " + str(self.X_))
        print("Y = " + str(self.Y_))
        print("Z = " + str(self.Z_))
        print("Roll = " + str(self.Roll_))
        print("Pitch = " + str(self.Pitch_))
        print("Yaw = " + str(self.Yaw_))

    def move_test_action(self):
        X_ = self.X_
        Y_ = self.Y_
        Z_ = self.Z_
        Roll_ = self.Roll_
        Pitch_ = self.Pitch_
        Yaw_ = self.Yaw_

        self.move_command_action(0, int(X_), int(Y_), 35000,
                                 int(Roll_), int(Pitch_), int(Yaw_))
        time.sleep(3)
        self.move_command_action(0, int(X_), int(Y_), int(Z_),
                                 int(Roll_), int(Pitch_), int(Yaw_))

    def set_byte_action(self, Data, Ins):
        # Ghi vị trí theo kiểu Robot Coordinate (Data type = 17)
        # Ghi vị trí theo kiểu Robot Pulse (Data type = 0)
        Data_Part = bytes([0x01, 0x00])
        Request_ID = bytes([0xFD])
        Command = bytes([0x7A, 0x00])
        Instance = Ins.to_bytes(2, byteorder='little', signed=True)
        Attribute = bytes([0x01])
        Service = bytes([0x02])
        Data = Data.to_bytes(1, byteorder='little', signed=True)

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        self.socket.received_function()

    def get_byte_action(self, Ins):
        # Ghi vị trí theo kiểu Robot Coordinate (Data type = 17)
        # Ghi vị trí theo kiểu Robot Pulse (Data type = 0)
        Data_Part = bytes([0x01, 0x00])
        Request_ID = bytes([0xFC])
        Command = bytes([0x7A, 0x00])
        Instance = Ins.to_bytes(2, byteorder='little', signed=True)
        Attribute = bytes([0x01])
        Service = bytes([0x01])
        Data = bytes([0x00])

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        frame_data = self.socket.received_function()

        value = frame_data[32]

        return value

    def set_position_action(self, X, Y, Z, Roll, Pitch, Yaw, Ins):
        Data_Part = bytes([0x2C, 0x00])
        Request_ID = bytes([0xFB])
        Command = bytes([0x7F, 0x00])
        Instance = Ins.to_bytes(2, byteorder='little', signed=True)
        Attribute = bytes([0x00])
        Service = bytes([0x02])

        Data_type = bytes([0x11, 0x00, 0x00, 0x00])
        Data_temp_1 = bytes([0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00])
        Data_X = X.to_bytes(4, byteorder='little', signed=True)
        Data_Y = Y.to_bytes(4, byteorder='little', signed=True)
        Data_Z = Z.to_bytes(4, byteorder='little', signed=True)
        Data_Roll = Roll.to_bytes(4, byteorder='little', signed=True)
        Data_Pitch = Pitch.to_bytes(4, byteorder='little', signed=True)
        Data_Yaw = Yaw.to_bytes(4, byteorder='little', signed=True)

        Data = []

        Data.extend(Data_type)
        Data.extend(Data_temp_1)
        Data.extend(Data_X)
        Data.extend(Data_Y)
        Data.extend(Data_Z)
        Data.extend(Data_Roll)
        Data.extend(Data_Pitch)
        Data.extend(Data_Yaw)

        Data = bytes(Data)

        frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

        self.socket.send_function(frame)
        self.socket.received_function()

    def get_position_action(self):
        Instance = None
        # Khi đọc vị trí thành phần Data trong Request là không có (0x00 0x00 0x00 0x00)
        # Khi đọc vị trí robot kiểu Pulse thì Instance = 1 (0x01 0x00)
        # Khi đọc vị trí robot kiểu Cartesian thì Instance = 101 (0x65 0x00)
        Data_Part = bytes([0x04, 0x00])
        Request_ID = bytes([0xFA])
        Command = bytes([0x75, 0x00])
        Attribute = bytes([0x00])
        Service = bytes([0x01])
        Data = bytes([0x00, 0x00, 0x00, 0x00])

        status = 0

        while status < 2:
            if status == 0:
                Instance = bytes([0x01, 0x00])

            elif status == 1:
                Instance = bytes([0x65, 0x00])

            frame = self.frame_data_send(Data_Part, Request_ID, Command, Instance, Attribute, Service, Data)

            self.socket.send_function(frame)
            frame_data = self.socket.received_function()

            first_axis = frame_data[52:56]
            first_axis_value = int.from_bytes(first_axis, byteorder='little', signed=True)
            second_axis = frame_data[56:60]
            second_axis_value = int.from_bytes(second_axis, byteorder='little', signed=True)
            third_axis = frame_data[60:64]
            third_axis_value = int.from_bytes(third_axis, byteorder='little', signed=True)
            fourth_axis = frame_data[64:68]
            fourth_axis_value = int.from_bytes(fourth_axis, byteorder='little', signed=True)
            fifth_axis = frame_data[68:72]
            fifth_axis_value = int.from_bytes(fifth_axis, byteorder='little', signed=True)
            sixth_axis = frame_data[72:76]
            sixth_axis_value = int.from_bytes(sixth_axis, byteorder='little', signed=True)

            if status == 0:
                self.uic.txt_S.setText(str(int(first_axis_value * 30 / 34816)))
                self.uic.txt_L.setText(str(int(second_axis_value * 90 / 102400)))
                self.uic.txt_U.setText(str(int(third_axis_value * 90 / 51200)))
                self.uic.txt_R.setText(str(int(fourth_axis_value * 30 / 10204)))
                self.uic.txt_B.setText(str(int(fifth_axis_value * 30 / 10204)))
                self.uic.txt_T.setText(str(int(sixth_axis_value * 30 / 10204)))
            elif status == 1:
                self.uic.txt_X.setText(str((first_axis_value / 1000)))
                self.uic.txt_Y.setText(str((second_axis_value / 1000)))
                self.uic.txt_Z.setText(str((third_axis_value / 1000)))
                self.uic.txt_Roll.setText(str((fourth_axis_value / 10000)))
                self.uic.txt_Pitch.setText(str((fifth_axis_value / 10000)))
                self.uic.txt_Yaw.setText(str((sixth_axis_value / 10000)))

            status += 1

    def serial_process_action(self, Pulse):
        Data = []
        STX = bytes([0x02])
        ACK = bytes([0x00])
        ETX = bytes([0x03])
        Command = bytes([0x50, 0x55, 0x4C, 0x53, 0x45])
        Service = self.method.to_bytes(4, byteorder='big', signed=True)
        Value = str(Pulse).encode()

        Data.extend(STX)
        Data.extend(Command)
        Data.extend(Service)
        Data.extend(Value)
        Data.extend(ACK)
        Data.extend(ETX)

        Data = bytes(Data)

        self.serial.sendSerial(Data)

    def update_frame(self, obj_detect, flag_last_id, color_frame, binary_frame, depth_frame, point_u, point_v, angle):
        print("Status detect: " + str(flag_last_id))
        print("u = " + str(point_u))
        print("v = " + str(point_v))
        print("angle = " + str(angle))

        rve = None
        tve = None

        if obj_detect:
            self.uic.lb_Object.setText("Object: True")
        else:
            self.uic.lb_Object.setText("Object: False")

        if flag_last_id:
            if self.flag_Enable_Job:
                self.flag_Data_Position = True
            else:
                self.flag_Data_Position = False
        else:
            self.flag_Data_Position = False

        if self.flag_RGB:
            frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)

            if self.flag_Capture_Background:
                save_path = 'D:\\A_Project\\PyCharm_Project'
                if not os.path.exists(save_path):
                    os.makedirs(save_path)
                file_name = 'Background.jpg'
                cv2.imwrite(os.path.join(save_path, file_name), color_frame)
                self.flag_Capture_Background = False
                print("Successful")

            if self.flag_Capture_Object:
                self.index_data += 1
                save_path_object = 'D:\\A_Project\\PyCharm_Project\\Object_Detection_2'
                if not os.path.exists(save_path_object):
                    os.makedirs(save_path_object)
                object_file_name = f'image_({self.index_data}).jpg'
                cv2.imwrite(os.path.join(save_path_object, object_file_name), color_frame)
                self.flag_Capture_Object = False
                self.uic.lb_Index.setText("Image_(" + str(self.index_data) + ")")

            # if self.flag_Conveyor:
            #     if (point_u != 0) & (point_v != 0):
            #         Xc, Yc, Zc, depth_value = self.camera.point(point_u, point_v)
            #
            #         Wc = np.array([[Xc],
            #                        [Yc],
            #                        [Zc],
            #                        [1.0]])
            #         Wr = self.T_cam2base.dot(Wc)
            #
            #         self.X_ = Wr[0][0] * 1000 + 5000
            #         self.Y_ = Wr[1][0] * 1000 + (-(Wr[1][0] * 1000)) - 50000
            #         self.Z_ = Wr[2][0] * 1000
            #         self.Roll_ = 180 * 10000
            #         self.Pitch_ = 0 * 10000
            #         self.Yaw_ = int(angle * 10000)
            #
            #         if obj_detect:
            #             print("\n\nX = " + str(self.X_) + '\n' +
            #                   "Y = " + str(self.Y_) + '\n' +
            #                   "Z = " + str(self.Z_) + '\n' +
            #                   "Roll = " + str(self.Roll_) + '\n' +
            #                   "Pitch = " + str(self.Pitch_) + '\n' +
            #                   "Yaw = " + str(self.Yaw_))

            if self.flag_Detect_ArUco:
                frame, rve, tve, _, u, v = ArUcoDetection.ARUCO_Detection(frame, self.Int_matrix, self.detector, 65)
                if self.flag_Capture_ArUco:
                    Xw = self.uic.txt_X.text()
                    Yw = self.uic.txt_Y.text()
                    Zw = self.uic.txt_Z.text()
                    Rx = self.uic.txt_Roll.text()
                    Ry = self.uic.txt_Pitch.text()
                    Rz = self.uic.txt_Yaw.text()

                    S_deg = int(self.uic.txt_S.text()) * 30 / 34816
                    L_deg = int(self.uic.txt_L.text()) * 90 / 102400
                    U_deg = int(self.uic.txt_U.text()) * 90 / 51200
                    R_deg = int(self.uic.txt_R.text()) * 30 / 10204
                    B_deg = int(self.uic.txt_B.text()) * 30 / 10204
                    T_deg = int(self.uic.txt_T.text()) * 30 / 10204

                    Xc, Yc, Zc, _ = self.camera.point(int(u), int(v))

                    self.index_data += 1

                    save_path_image = (f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Data'
                                       f'\\Image\\Data_({self.index_folder})')
                    if not os.path.exists(save_path_image):
                        os.makedirs(save_path_image)
                    image_file_name = f'image_({self.index_data}).jpg'
                    cv2.imwrite(os.path.join(save_path_image, image_file_name), color_frame)

                    save_path_txt = (f'D:\\A_Project_DK-TDH\\PyCharm_Project\\Camera_Calibration\\Data'
                                     f'\\Position\\Data_({self.index_folder})')
                    if not os.path.exists(save_path_txt):
                        os.makedirs(save_path_txt)
                    txt_file_name = f'position_({self.index_data}).txt'
                    txt_file_path = os.path.join(save_path_txt, txt_file_name)
                    with open(txt_file_path, 'w') as txt_file:
                        txt_file.write(str(u) + '\n' + str(v) + '\n' +
                                       str(Xc) + '\n' + str(Yc) + '\n' + str(Zc) + '\n' +
                                       Xw + '\n' + Yw + '\n' + Zw + '\n' +
                                       Rx + '\n' + Ry + '\n' + Rz + '\n' +
                                       str(S_deg) + '\n' + str(L_deg) + '\n' + str(U_deg) + '\n' +
                                       str(R_deg) + '\n' + str(B_deg) + '\n' + str(T_deg))
                    self.flag_ArUco = False
                    self.uic.lb_Index.setText("Image_(" + str(self.index_data) + ")")

            elif self.camera.flag_Detect_COLOR:
                if self.flag_Data_Position:
                    if (point_u != 0) & (point_v != 0):
                        Xc, Yc, Zc, depth_value = self.camera.point(point_u, point_v)

                        Wc = np.array([[Xc],
                                       [Yc],
                                       [Zc],
                                       [1.0]])
                        Wr = self.T_cam2base.dot(Wc)

                        self.X_ = Wr[0][0] * 1000
                        self.Y_ = Wr[1][0] * 1000
                        self.Z_ = Wr[2][0] * 1000
                        self.Roll_ = 180 * 10000
                        self.Pitch_ = 0 * 10000
                        self.Yaw_ = 0 * 10000

                        print("X = " + str(self.X_))
                        print("Y = " + str(self.Y_))
                        print("Z = " + str(self.Z_))
                        print("Roll = " + str(self.Roll_))
                        print("Pitch = " + str(self.Pitch_))
                        print("Yaw = " + str(self.Yaw_))

                        self.flag_Data_Position = False

                        if self.flag_Process:
                            self.move_job_action()
                            self.flag_Process = False

            elif self.camera.flag_Detect_YOLO:
                if self.flag_Data_Position:
                    if (point_u != 0) & (point_v != 0):
                        Xc, Yc, Zc, depth_value = self.camera.point(point_u, point_v)

                        Wc = np.array([[Xc],
                                       [Yc],
                                       [Zc],
                                       [1.0]])
                        Wr = self.T_cam2base.dot(Wc)

                        # if Wr[0][0] > 200 & Wr[0][0] < 280 * Wr[1][0] > (-190) & Wr[1][0] < 10:
                        self.X_ = Wr[0][0] * 1000 + 15000
                        self.Y_ = Wr[1][0] * 1000 + 73000
                        self.Z_ = Wr[2][0] * 1000 - 5000
                        self.Roll_ = 180 * 10000
                        self.Pitch_ = 0 * 10000
                        self.Yaw_ = int(angle * 10000)

                        print("X = " + str(self.X_))
                        print("Y = " + str(self.Y_))
                        print("Z = " + str(self.Z_))
                        print("Roll = " + str(self.Roll_))
                        print("Pitch = " + str(self.Pitch_))
                        print("Yaw = " + str(self.Yaw_))

                        self.flag_Data_Position = False

                        if self.flag_Process:
                            if self.flag_Enable_Job:
                                self.move_job_action()
                                self.flag_Process = False

            pixmap = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            self.uic.Display_frame.setPixmap(QPixmap.fromImage(pixmap))

        elif self.flag_Binary:
            frame = binary_frame

            # Ảnh Binary thì dùng Format Indexed8
            pixmap = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_Indexed8)
            self.uic.Display_frame.setPixmap(QPixmap.fromImage(pixmap))

        elif self.flag_Depth:
            frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2RGB)

            pixmap = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            self.uic.Display_frame.setPixmap(QPixmap.fromImage(pixmap))

        else:
            self.uic.Display_frame.clear()

    def update_timer(self):
        if self.flag_Select:
            self.get_position_action()
            self.flag_Select = False

        elif not self.flag_Select:
            if self.flag_Enable_Job:
                status = self.get_byte_action(5)
                self.uic.txt_Status.setText("Byte 5 = " + str(status))
                if status == 0:
                    self.flag_Process = False
                    self.uic.lb_Gripper.setText("Gripper: None")
                elif status == 1:
                    if self.flag_Auto:
                        self.flag_Process = True
                    else:
                        self.flag_Process = False
                elif status == 2:
                    self.uic.lb_Gripper.setText("Gripper: Pick")
                    self.method = 4
                    self.serial_process_action(30)
                    self.set_byte_action(0, 5)
                    self.timer.stop()
                elif status == 3:
                    self.uic.lb_Gripper.setText("Gripper: Place")
                    self.method = 5
                    self.serial_process_action(80)
                    self.set_byte_action(0, 5)
                    self.timer.stop()
            else:
                self.method = 0
                self.serial_process_action(80)
            self.flag_Select = True

    def update_serial(self, Data_received):
        # self.uic.txt_Received.setText(str(Data_received))
        Service = Data_received[9]
        if Service == b'\x04':
            self.set_byte_action(1, 3)
            self.flag_Select = True
            self.timer.start(50)
        elif Service == b'\x05':
            self.set_byte_action(1, 1)
            self.flag_Select = True
            self.timer.start(50)
        self.method = 0

    # def update_mode(self):
    #     self.Mode_Control = self.uic.cBox_Mode.currentText()
    #     print("Mode control: " + self.Mode_Control)

    # def conveyor_action(self):
    #     if self.uic.btn_Conveyor.text() == "Conveyor ON":
    #         self.conveyor.send_function(bytes([0x01]))
    #         # frame = bytes([0x11,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00, 0x00,
    #         #                0x00, 0x00, 0x00, 0x00,
    #         #                0x00, 0x08,
    #         #                0x20,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00, 0x00,
    #         #                0x00, 0x01])
    #         # self.conveyor.send_function(frame)
    #         # self.conveyor.received_function()
    #         self.uic.btn_Conveyor.setText("Conveyor OFF")
    #     elif self.uic.btn_Conveyor.text() == "Conveyor OFF":
    #         # frame = bytes([0x11,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00, 0x00,
    #         #                0x00, 0x00, 0x00, 0x00,
    #         #                0x00, 0x08,
    #         #                0x20,
    #         #                0x00,
    #         #                0x00,
    #         #                0x00, 0x00,
    #         #                0x00, 0x00])
    #         # self.conveyor.send_function(frame)
    #         # self.conveyor.received_function()
    #         self.uic.btn_Conveyor.setText("Conveyor ON")


class RobotSocket(QThread):
    def __init__(self):
        super(RobotSocket, self).__init__()
        self.IP = ''
        self.Port = 0

        # SOCK_STREAM là dùng để truyền thông bằng giao thức TCP
        # SOCK_DGRAM là dùng để truyền thông bằng giao thức UDP
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Robot Socket Finished Init")

    def run(self):
        # Thiết lập kết nối
        try:
            # print("Status Robot Connected")
            # Đối số của lệnh self.s.bind() là một đối số kiểu tuple (Tương tự như list hay array) chứa IP và Port
            # print("IP: " + str(self.IP) + '\n' + "Port: " + str(int(self.Port)))
            self.s.connect((self.IP, self.Port))
            print("\nStatus Robot Connected")
            print("IP: " + self.IP + '\n' + "Port: " + str(self.Port))
            self.flag_connected = True
        except Exception as e:
            print("Error:", e)

    def disconnect(self):
        if self.s:
            self.s.close()
            self.flag_connected = False
            print("Status Robot Disconnected")
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def robot_address(self, IP, Port):
        self.IP = IP
        self.Port = Port

    def send_function(self, frame_data):
        # Identifier            (4byte) (0 1 2 3)
        # Header                (2byte) (4 5)
        # Data part size        (2byte) (6 7)
        # Reserve 1             (1byte) (8)
        # Processing division   (1byte) (9)
        # ACK                   (1byte) (10)
        # Request ID            (1byte) (11)
        # Block No              (4byte) (12 13 14 15)
        # Reserve 2             (8byte) (16 17 18 19 20 21 22 23)
        # Command               (2byte) (24 25)
        # Instance              (2byte) (26 27)
        # Attribute             (1byte) (28)
        # Service               (1byte) (29)
        # Padding               (2byte) (30 31)
        # Data                  (4byte) (32 33 34 35)

        self.s.sendto(frame_data, (self.IP, self.Port))
        print("Data Send: " + str(frame_data))

    def received_function(self):
        # Identifier            (4byte) (0 1 2 3)
        # Header                (2byte) (4 5)
        # Data part size        (2byte) (6 7)
        # Reserve 1             (1byte) (8)
        # Processing division   (1byte) (9)
        # ACK                   (1byte) (10)
        # Request ID            (1byte) (11)
        # Block No              (4byte) (12 13 14 15)
        # Reserve 2             (8byte) (16 17 18 19 20 21 22 23)
        # Service               (1byte) (24)
        # Status                (1byte) (25)
        # Add status size       (1byte) (26)
        # Padding               (1byte) (27)
        # Add status            (2byte) (28 29)
        # Padding               (2byte) (30 31)
        # Data part             (4byte) (32 33 34 35)

        frame_data = self.s.recv(1024)
        print("Data Received: " + str(frame_data) + '\n')
        return frame_data


class CameraThread(QThread):
    image = pyqtSignal(bool, bool, np.ndarray, np.ndarray, np.ndarray, int, int, float)

    def __init__(self):
        super(CameraThread, self).__init__()
        self.rs = RealsenseCamera()
        self.yoloDetect = YoloDetection()
        self.yoloSegment = YoloSegmentation()
        print("Camera Finished Init")

    def run(self):
        binary_frame = None
        obj = None
        flag_last_id = False
        point_center = None
        u = 0
        v = 0
        angle = 0

        width = 640
        height = 480
        black_frame = np.zeros((height, width), dtype=np.uint8)

        while self.running:
            ret, color_frame, depth_frame, _, _, _ = self.rs.get_frame_stream()

            if ret:
                # Vẽ vùng làm việc
                mean = 200
                min_X = 100
                max_X = 300
                min_Y = 275
                max_Y = 420

                tl_point = (min_X, min_Y)
                br_point = (max_X, max_Y)
                cv2.rectangle(color_frame, tl_point, br_point, (0, 0, 0), 1)

                binary_float32 = self.yoloSegment.getSegment(color_frame)

                if binary_float32 is None:
                    binary_frame = black_frame
                    obj = False
                else:
                    binary_normalized = cv2.normalize(binary_float32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    binary_frame = binary_normalized.astype(np.uint8)
                    obj = True

                if self.flag_Detect_YOLO:
                    color_frame, last_id, _, point_center, top_left, bottom_right, _ = self.yoloDetect.getObject(
                        color_frame)

                    if last_id is None:
                        flag_last_id = False

                    else:
                        if point_center[0] > mean:
                            contour_box, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            for c in contour_box:
                                area = cv2.contourArea(c)
                                if area < 1000:
                                    continue
                                else:
                                    M = cv2.moments(c)
                                    cX = int(M["m10"] / M["m00"])
                                    cY = int(M["m01"] / M["m00"])

                                    if (cX > min_X) & (cX < max_X) & (cY > min_Y) & (cY < max_Y):
                                        angle, _ = self.getOrientation(c, color_frame)
                                        angle = (angle * 180 / math.pi) + 90.0

                                        if angle > 90.0:
                                            angle = angle - 180

                            flag_last_id = True

                        else:
                            flag_last_id = False

                elif self.flag_Detect_COLOR:
                    color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
                    color_frame, u, v = ColorDetection.COLOR_objectdetection(color_frame)
                    color_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
                    if (u is None) & (v is None):
                        u = 0
                        v = 0

                else:
                    flag_last_id = False

                if flag_last_id:
                    u = point_center[0]
                    v = point_center[1]
                else:
                    u = 0
                    v = 0
                    angle = 0

                self.image.emit(obj, flag_last_id, color_frame, binary_frame, depth_frame, u, v, angle)

    def matrix(self):
        _, _, _, matrix, _, dist = self.rs.get_frame_stream()
        depth_intrinsic = np.array([[matrix.fx, 0, matrix.ppx],
                                    [0, matrix.fy, matrix.ppy],
                                    [0, 0, 1]])
        return depth_intrinsic, dist

    def point(self, u, v):
        X, Y, Z, depth = self.rs.pixel_to_point(u, v)
        return X, Y, Z, depth

    def drawAxis(self, img, p_, q_, colour, scale):
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

    def getOrientation(self, pts, img):
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
        # cv2.circle(img, cntr, 3, (255, 0, 255), 2)

        p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0],
              cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0],
              cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])

        self.drawAxis(img, cntr, p1, (255, 0, 0), 3)
        self.drawAxis(img, cntr, p2, (255, 255, 255), 7)

        angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

        if cntr[0] < p1[0]:
            angle = -angle

        return angle, cntr


class SerialProcess(QThread):
    message = pyqtSignal(list)

    def __init__(self):
        super(SerialProcess, self).__init__()
        self.serialPort = serial.Serial()
        self.Data_received = []
        self.flag_DataAvailable = False
        self.flag_Connected = None
        self.STX = bytes([0x02])
        self.ETX = bytes([0x03])
        print("Serial Port Finished Init")

    def serial_connect(self, Port):
        self.serialPort.port = Port
        self.serialPort.open()
        self.flag_Connected = True
        print("Serial Port is Open")

    def serial_disconnect(self):
        self.flag_Connected = False
        print("Serial Port is Close")

    def run(self):
        while True:
            if self.flag_Connected:
                Data = self.serialPort.read()
                if Data == self.STX:
                    self.flag_DataAvailable = True
                elif Data == self.ETX:
                    self.Data_received.append(Data)
                    self.flag_DataAvailable = False
                if self.flag_DataAvailable:
                    self.Data_received.append(Data)
                else:
                    # print("Data received = " + str(self.Data_received) + '\n' +
                    #       "Data [9] = " + str(self.Data_received[9]) + '\n' +
                    #       "Data [12] = " + str(self.Data_received[12]))
                    self.message.emit(self.Data_received)
                    self.Data_received.clear()
            else:
                self.serialPort.close()
                break

    def sendSerial(self, Data):
        print(str(Data))
        self.serialPort.write(Data)


# class ConveyorSocket(QThread):
#     def __init__(self):
#         super(ConveyorSocket, self).__init__()
#         self.IP = '192.168.1.1'
#         self.Port = 10001
#
#         # SOCK_STREAM là dùng để truyền thông bằng giao thức TCP
#         # SOCK_DGRAM là dùng để truyền thông bằng giao thức UDP
#         self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         # self.client = ModbusTcpClient(self.IP, port=self.Port)
#         print("Conveyor Socket Finished Init")
#
#     def run(self):
#         # Thiết lập kết nối
#         try:
#             self.s.connect((self.IP, self.Port))
#             # self.client.connect()
#             print("IP: " + self.IP + '\n' + "Port: " + str(self.Port))
#         except Exception as e:
#             print("Error:", e)
#
#     def disconnect(self):
#         if self.s:
#             self.s.close()
#             print("Status Conveyor Disconnected")
#             self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         # print("Status Conveyor Disconnected")
#
#     def send_function(self, data):
#         self.s.sendto(data, (self.IP, self.Port))
#         print("Data Send: " + str(data))
#         # response = self.client.write_register(0, 1, (self.IP, self.Port))
#         # print(str(response))
#         # if response.isError():
#         #     print("Error:", response)
#         # else:
#         #     print("Successfully")
#
#     def received_function(self):
#         frame_data, _ = self.s.recvfrom(4096)
#         print("Data Received: " + str(frame_data) + '\n')


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())
