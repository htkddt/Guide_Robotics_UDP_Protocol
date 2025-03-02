import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QPushButton, QTextEdit, QSlider, QLabel, QLineEdit
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGroupBox
from PyQt5.QtWidgets import QMessageBox, QSizePolicy, QFrame
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QUrl, QSize, QRect, QCoreApplication, QPropertyAnimation, pyqtSignal


class MainWindowUI(object):
    def initUI(self, mainWindow):
        mainWindow.setObjectName("MainWindow")
        mainWindow.resize(1520, 900)
        mainWindow.setWindowTitle("Robot Application")

        self.defaultFontBold = QFont("Time New Roman", 8)
        self.defaultFontBold.setBold(True)
        self.defaultFontRegular = QFont("Time New Roman", 8)
        self.defaultFontRegular.setBold(False)

        self.initConnectionGroup(mainWindow)
        self.initBasicControlGroup(mainWindow)
        self.initIncrementalControlGroup(mainWindow)
        self.initGetPositionGroup(mainWindow)
        self.initJobControlGroup(mainWindow)
        self.initCameraGroup(mainWindow)

    def initConnectionGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grConnection = QGroupBox(mw)
        grConnection.setTitle("CONNECTION")
        grConnection.setFont(self.defaultFontBold)
        grConnection.setGeometry(30, 10, 561, 251)

        # Group Robot inside Group Connection
        grRobot = QGroupBox(grConnection)
        grRobot.setTitle("ROBOT")
        grRobot.setGeometry(20, 20, 251, 221)

        lbIPRobot = QLabel("IP Address", grRobot)
        lbIPRobot.setFont(self.defaultFontRegular)
        lbIPRobot.setAlignment(Qt.AlignLeft)
        lbIPRobot.setGeometry(10, 30, 55, 16)

        self.txtIPRobot = QLineEdit("192.168.1.16", grRobot)
        self.txtIPRobot.setAlignment(Qt.AlignLeft)
        self.txtIPRobot.setGeometry(80, 30, 161, 22)

        lbPortRobot = QLabel("Port", grRobot)
        lbPortRobot.setFont(self.defaultFontRegular)
        lbPortRobot.setAlignment(Qt.AlignLeft)
        lbPortRobot.setGeometry(10, 60, 55, 16)

        self.txtPortRobot = QLineEdit("10040", grRobot)
        self.txtPortRobot.setAlignment(Qt.AlignLeft)
        self.txtPortRobot.setGeometry(80, 60, 161, 22)

        self.btnRobotConDis = QPushButton("Connect", grRobot)
        self.btnRobotConDis.setFont(fontBold)
        self.btnRobotConDis.setGeometry(10, 90, 231, 41)

        lbDeviceRobotStatus = QLabel("Device Robot:", grRobot)
        lbDeviceRobotStatus.setFont(self.defaultFontRegular)
        lbDeviceRobotStatus.setAlignment(Qt.AlignLeft)
        lbDeviceRobotStatus.setGeometry(10, 150, 131, 20)

        self.btnDeviceRobot = SwitchButton(mw, grRobot, 150, 142)

        lbRobotConnectStatus = QLabel("Connection Robot:", grRobot)
        lbRobotConnectStatus.setFont(self.defaultFontRegular)
        lbRobotConnectStatus.setAlignment(Qt.AlignLeft)
        lbRobotConnectStatus.setGeometry(10, 190, 131, 20)

        self.txtRobotConnectStatus = QLabel("Disconnected", grRobot)
        self.txtRobotConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtRobotConnectStatus.setGeometry(140, 190, 91, 20)

        # Group Gripper inside Group Connection
        grGripper = QGroupBox(grConnection)
        grGripper.setTitle("GRIPPER")
        grGripper.setGeometry(290, 20, 251, 221)

        lbPortGripper = QLabel("Port", grGripper)
        lbPortGripper.setFont(self.defaultFontRegular)
        lbPortGripper.setAlignment(Qt.AlignLeft)
        lbPortGripper.setGeometry(10, 30, 55, 16)

        self.txtPortGripper = QLineEdit("COM4", grGripper)
        self.txtPortGripper.setAlignment(Qt.AlignLeft)
        self.txtPortGripper.setGeometry(80, 30, 161, 22)

        lbBaudGripper = QLabel("Baudrate", grGripper)
        lbBaudGripper.setFont(self.defaultFontRegular)
        lbBaudGripper.setAlignment(Qt.AlignLeft)
        lbBaudGripper.setGeometry(10, 60, 55, 16)

        self.txtBaudGripper = QLineEdit("115200", grGripper)
        self.txtBaudGripper.setAlignment(Qt.AlignLeft)
        self.txtBaudGripper.setGeometry(80, 60, 161, 22)

        self.btnGripperConDis = QPushButton("Connect", grGripper)
        self.btnGripperConDis.setFont(fontBold)
        self.btnGripperConDis.setGeometry(10, 90, 231, 41)

        lbDeviceIOStatus = QLabel("Device I/O:", grGripper)
        lbDeviceIOStatus.setFont(self.defaultFontRegular)
        lbDeviceIOStatus.setAlignment(Qt.AlignLeft)
        lbDeviceIOStatus.setGeometry(10, 150, 131, 20)

        self.btnDeviceIO = SwitchButton(mw, grGripper, 150, 142)

        lbIOConnectStatus = QLabel("Connection I/O:", grGripper)
        lbIOConnectStatus.setFont(self.defaultFontRegular)
        lbIOConnectStatus.setAlignment(Qt.AlignLeft)
        lbIOConnectStatus.setGeometry(10, 190, 131, 20)

        self.txtIOConnectStatus = QLabel("Disconnected", grGripper)
        self.txtIOConnectStatus.setFont(self.defaultFontBold)
        self.txtIOConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtIOConnectStatus.setGeometry(140, 190, 91, 20)

    def initBasicControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grBasicControl = QGroupBox(mw)
        grBasicControl.setTitle("BASIC CONTROL")
        grBasicControl.setFont(self.defaultFontBold)
        grBasicControl.setGeometry(30, 280, 561, 131)

        self.btnAutoMode = QPushButton("Auto", grBasicControl)
        self.btnAutoMode.setGeometry(30, 30, 81, 41)

        self.btnManualMode = QPushButton("Manual", grBasicControl)
        self.btnManualMode.setGeometry(130, 30, 101, 41)

        self.btnServo = QPushButton("Servo On", grBasicControl)
        self.btnServo.setGeometry(250, 30, 121, 41)

        self.txtSeralValue = QLineEdit(grBasicControl)
        self.txtSeralValue.setGeometry(390, 30, 151, 41)

        lbMode = QLabel("MODE", grBasicControl)
        lbMode.setAlignment(Qt.AlignCenter)
        lbMode.setFont(fontRegular)
        lbMode.setGeometry(40, 80, 61, 41)

        self.txtModeValue = QLabel("NONE", grBasicControl)
        self.txtModeValue.setFont(fontRegular)
        self.txtModeValue.setAlignment(Qt.AlignCenter)
        self.txtModeValue.setGeometry(130, 80, 101, 41)

        self.btnHome = QPushButton("Go Home", grBasicControl)
        self.btnHome.setGeometry(250, 80, 121, 41)

        self.btnSerialSend = QPushButton("Send Data Serial", grBasicControl)
        self.btnSerialSend.setGeometry(390, 80, 151, 41)

    def initIncrementalControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grIncremental = QGroupBox(mw)
        grIncremental.setTitle("INCREMENTAL POSITION")
        grIncremental.setFont(self.defaultFontBold)
        grIncremental.setAlignment(Qt.AlignLeft)
        grIncremental.setGeometry(30, 430, 561, 421)

        lbX = QLabel("X", grIncremental)
        lbX.setFont(self.defaultFontRegular)
        lbX.setGeometry(30, 40, 31, 20)

        self.btnXInc = QPushButton("+", grIncremental)
        self.btnXInc.setGeometry(80, 30, 81, 41)
        self.btnXDec = QPushButton("-", grIncremental)
        self.btnXDec.setGeometry(180, 30, 81, 41)

        lbY = QLabel("Y", grIncremental)
        lbY.setFont(self.defaultFontRegular)
        lbY.setGeometry(30, 100, 31, 20)

        self.btnYInc = QPushButton("+", grIncremental)
        self.btnYInc.setGeometry(80, 90, 81, 41)
        self.btnYDec = QPushButton("-", grIncremental)
        self.btnYDec.setGeometry(180, 90, 81, 41)

        lbZ = QLabel("Z", grIncremental)
        lbZ.setFont(self.defaultFontRegular)
        lbZ.setGeometry(30, 160, 31, 20)

        self.btnZInc = QPushButton("+", grIncremental)
        self.btnZInc.setGeometry(80, 150, 81, 41)
        self.btnZDec = QPushButton("-", grIncremental)
        self.btnZDec.setGeometry(180, 150, 81, 41)

        lbRoll = QLabel("Roll", grIncremental)
        lbRoll.setFont(self.defaultFontRegular)
        lbRoll.setGeometry(30, 250, 31, 20)

        self.btnRollInc = QPushButton("+", grIncremental)
        self.btnRollInc.setGeometry(80, 240, 81, 41)
        self.btnRollDec = QPushButton("-", grIncremental)
        self.btnRollDec.setGeometry(180, 240, 81, 41)

        lbPitch = QLabel("Pitch", grIncremental)
        lbPitch.setFont(self.defaultFontRegular)
        lbPitch.setGeometry(30, 310, 31, 20)

        self.btnPitchInc = QPushButton("+", grIncremental)
        self.btnPitchInc.setGeometry(80, 300, 81, 41)
        self.btnPitchDec = QPushButton("-", grIncremental)
        self.btnPitchDec.setGeometry(180, 300, 81, 41)

        lbYaw = QLabel("Yaw", grIncremental)
        lbYaw.setFont(self.defaultFontRegular)
        lbYaw.setGeometry(30, 370, 31, 20)

        self.btnYawInc = QPushButton("+", grIncremental)
        self.btnYawInc.setGeometry(80, 360, 81, 41)
        self.btnYawDec = QPushButton("-", grIncremental)
        self.btnYawDec.setGeometry(180, 360, 81, 41)

        lbDis = QLabel("DISTANCE", grIncremental)
        lbDis.setFont(fontBold)
        lbDis.setGeometry(310, 29, 141, 31)

        self.sliderDistanceMM = QSlider(grIncremental)
        self.sliderDistanceMM.setGeometry(330, 70, 111, 22)
        self.sliderDistanceMM.setMinimum(0)
        self.sliderDistanceMM.setMaximum(10000)
        self.sliderDistanceMM.setSingleStep(1)
        self.sliderDistanceMM.setOrientation(Qt.Horizontal)

        self.txtDisMM = QLabel("0", grIncremental)
        self.txtDisMM.setAlignment(Qt.AlignCenter)
        self.txtDisMM.setFont(self.defaultFontRegular)
        self.txtDisMM.setGeometry(450, 70, 41, 20)

        lbMM = QLabel("mm", grIncremental)
        lbMM.setAlignment(Qt.AlignCenter)
        lbMM.setFont(self.defaultFontRegular)
        lbMM.setGeometry(500, 70, 41, 20)

        self.sliderDistanceDEG = QSlider(grIncremental)
        self.sliderDistanceDEG.setGeometry(330, 120, 111, 22)
        self.sliderDistanceDEG.setMinimum(0)
        self.sliderDistanceDEG.setMaximum(100000)
        self.sliderDistanceDEG.setSingleStep(1)
        self.sliderDistanceDEG.setOrientation(Qt.Horizontal)

        self.txtDisDEG = QLabel("0", grIncremental)
        self.txtDisDEG.setAlignment(Qt.AlignCenter)
        self.txtDisDEG.setFont(self.defaultFontRegular)
        self.txtDisDEG.setGeometry(450, 120, 41, 20)

        lbDEG = QLabel("deg", grIncremental)
        lbDEG.setAlignment(Qt.AlignCenter)
        lbDEG.setFont(self.defaultFontRegular)
        lbDEG.setGeometry(500, 120, 41, 20)

        lbSpeManual = QLabel("SPEED MANUAL", grIncremental)
        lbSpeManual.setFont(fontBold)
        lbSpeManual.setGeometry(310, 169, 141, 31)

        self.sliderSpeedMM = QSlider(grIncremental)
        self.sliderSpeedMM.setGeometry(330, 210, 111, 22)
        self.sliderSpeedMM.setMinimum(0)
        self.sliderSpeedMM.setMaximum(300)
        self.sliderSpeedMM.setSingleStep(1)
        self.sliderSpeedMM.setOrientation(Qt.Horizontal)

        self.txtSpeMM = QLabel("0", grIncremental)
        self.txtSpeMM.setAlignment(Qt.AlignCenter)
        self.txtSpeMM.setFont(self.defaultFontRegular)
        self.txtSpeMM.setGeometry(450, 210, 41, 20)

        lbMMs = QLabel("mm/s", grIncremental)
        lbMMs.setAlignment(Qt.AlignCenter)
        lbMMs.setFont(self.defaultFontRegular)
        lbMMs.setGeometry(500, 210, 41, 20)

        self.sliderSpeedDEG = QSlider(grIncremental)
        self.sliderSpeedDEG.setGeometry(330, 260, 111, 22)
        self.sliderSpeedDEG.setMinimum(0)
        self.sliderSpeedDEG.setMaximum(1000)
        self.sliderSpeedDEG.setSingleStep(1)
        self.sliderSpeedDEG.setOrientation(Qt.Horizontal)

        self.txtSpeDEG = QLabel("0", grIncremental)
        self.txtSpeDEG.setAlignment(Qt.AlignCenter)
        self.txtSpeDEG.setFont(self.defaultFontRegular)
        self.txtSpeDEG.setGeometry(450, 260, 41, 20)

        lbDEGs = QLabel("deg/s", grIncremental)
        lbDEGs.setAlignment(Qt.AlignCenter)
        lbDEGs.setFont(self.defaultFontRegular)
        lbDEGs.setGeometry(500, 260, 41, 20)

        lbSpeAuto = QLabel("SPEED AUTO", grIncremental)
        lbSpeAuto.setFont(fontBold)
        lbSpeAuto.setGeometry(310, 320, 141, 31)

        self.sliderSpeedAuto = QSlider(grIncremental)
        self.sliderSpeedAuto.setGeometry(330, 370, 111, 22)
        self.sliderSpeedAuto.setMinimum(0)
        self.sliderSpeedAuto.setMaximum(300)
        self.sliderSpeedAuto.setSingleStep(1)
        self.sliderSpeedAuto.setOrientation(Qt.Horizontal)

        self.txtSpeAuto = QLabel("0", grIncremental)
        self.txtSpeAuto.setAlignment(Qt.AlignCenter)
        self.txtSpeAuto.setFont(self.defaultFontRegular)
        self.txtSpeAuto.setGeometry(450, 370, 41, 20)

        lbMMsAuto = QLabel("mm/s", grIncremental)
        lbMMsAuto.setAlignment(Qt.AlignCenter)
        lbMMsAuto.setFont(self.defaultFontRegular)
        lbMMsAuto.setGeometry(500, 370, 41, 20)

    def initGetPositionGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grGetPosition = QGroupBox(mw)
        grGetPosition.setTitle("GET POSITION")
        grGetPosition.setFont(self.defaultFontBold)
        grGetPosition.setGeometry(610, 10, 481, 301)

        lbS = QLabel("S", grGetPosition)
        lbS.setFont(self.defaultFontRegular)
        lbS.setGeometry(20, 60, 21, 20)

        lbL = QLabel("L", grGetPosition)
        lbL.setFont(self.defaultFontRegular)
        lbL.setGeometry(20, 90, 21, 20)

        lbU = QLabel("U", grGetPosition)
        lbU.setFont(self.defaultFontRegular)
        lbU.setGeometry(20, 120, 21, 20)

        lbR = QLabel("R", grGetPosition)
        lbR.setFont(self.defaultFontRegular)
        lbR.setGeometry(20, 150, 21, 20)

        lbB = QLabel("B", grGetPosition)
        lbB.setFont(self.defaultFontRegular)
        lbB.setGeometry(20, 180, 21, 20)

        lbT = QLabel("T", grGetPosition)
        lbT.setFont(self.defaultFontRegular)
        lbT.setGeometry(20, 210, 21, 20)

        for i in range(6):
            lbDeg = QLabel("deg", grGetPosition)
            lbDeg.setFont(self.defaultFontRegular)
            lbDeg.setGeometry(200, (60 + 30 * i), 31, 20)

        lbX = QLabel("X", grGetPosition)
        lbX.setFont(self.defaultFontRegular)
        lbX.setAlignment(Qt.AlignCenter)
        lbX.setGeometry(250, 60, 31, 20)

        lbY = QLabel("Y", grGetPosition)
        lbY.setFont(self.defaultFontRegular)
        lbY.setAlignment(Qt.AlignCenter)
        lbY.setGeometry(250, 90, 31, 20)

        lbZ = QLabel("Z", grGetPosition)
        lbZ.setFont(self.defaultFontRegular)
        lbZ.setAlignment(Qt.AlignCenter)
        lbZ.setGeometry(250, 120, 31, 20)

        lbRoll = QLabel("Roll", grGetPosition)
        lbRoll.setFont(self.defaultFontRegular)
        lbRoll.setAlignment(Qt.AlignCenter)
        lbRoll.setGeometry(250, 150, 31, 20)

        lbPitch = QLabel("Pitch", grGetPosition)
        lbPitch.setFont(self.defaultFontRegular)
        lbPitch.setAlignment(Qt.AlignCenter)
        lbPitch.setGeometry(250, 180, 31, 20)

        lbYaw = QLabel("Yaw", grGetPosition)
        lbYaw.setFont(self.defaultFontRegular)
        lbYaw.setAlignment(Qt.AlignCenter)
        lbYaw.setGeometry(250, 210, 31, 20)

        for i in range(6):
            if i < 3:
                lbMM = QLabel("mm", grGetPosition)
                lbMM.setFont(self.defaultFontRegular)
                lbMM.setGeometry(440, (60 + 30 * i), 31, 20)
            else:
                lbDeg = QLabel("deg", grGetPosition)
                lbDeg.setFont(self.defaultFontRegular)
                lbDeg.setGeometry(440, (60 + 30 * i), 31, 20)

        self.btnGetPosition = QPushButton("Get Position", grGetPosition)
        self.btnGetPosition.setFont(fontBold)
        self.btnGetPosition.setGeometry(20, 250, 441, 41)

        grPulse = QGroupBox("PULSE", grGetPosition)
        grPulse.setFont(self.defaultFontBold)
        grPulse.setAlignment(Qt.AlignCenter)
        grPulse.setGeometry(50, 30, 141, 211)

        self.txtS = QLineEdit(grPulse)
        self.txtS.setAlignment(Qt.AlignCenter)
        self.txtS.setGeometry(10, 30, 121, 22)
        self.txtL = QLineEdit(grPulse)
        self.txtL.setAlignment(Qt.AlignCenter)
        self.txtL.setGeometry(10, 60, 121, 22)
        self.txtU = QLineEdit(grPulse)
        self.txtU.setAlignment(Qt.AlignCenter)
        self.txtU.setGeometry(10, 90, 121, 22)
        self.txtR = QLineEdit(grPulse)
        self.txtR.setAlignment(Qt.AlignCenter)
        self.txtR.setGeometry(10, 120, 121, 22)
        self.txtB = QLineEdit(grPulse)
        self.txtB.setAlignment(Qt.AlignCenter)
        self.txtB.setGeometry(10, 150, 121, 22)
        self.txtT = QLineEdit(grPulse)
        self.txtT.setAlignment(Qt.AlignCenter)
        self.txtT.setGeometry(10, 180, 121, 22)

        grCartesian = QGroupBox("CARTESIAN", grGetPosition)
        grCartesian.setFont(self.defaultFontBold)
        grCartesian.setAlignment(Qt.AlignCenter)
        grCartesian.setGeometry(290, 30, 141, 211)

        self.txtX = QLineEdit(grCartesian)
        self.txtX.setAlignment(Qt.AlignCenter)
        self.txtX.setGeometry(10, 30, 121, 22)
        self.txtY = QLineEdit(grCartesian)
        self.txtY.setAlignment(Qt.AlignCenter)
        self.txtY.setGeometry(10, 60, 121, 22)
        self.txtZ = QLineEdit(grCartesian)
        self.txtZ.setAlignment(Qt.AlignCenter)
        self.txtZ.setGeometry(10, 90, 121, 22)
        self.txtRoll = QLineEdit(grCartesian)
        self.txtRoll.setAlignment(Qt.AlignCenter)
        self.txtRoll.setGeometry(10, 120, 121, 22)
        self.txtPitch = QLineEdit(grCartesian)
        self.txtPitch.setAlignment(Qt.AlignCenter)
        self.txtPitch.setGeometry(10, 150, 121, 22)
        self.txtYaw = QLineEdit(grCartesian)
        self.txtYaw.setAlignment(Qt.AlignCenter)
        self.txtYaw.setGeometry(10, 180, 121, 22)

    def initJobControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grJobControl = QGroupBox(mw)
        grJobControl.setTitle("JOB CONTROL")
        grJobControl.setFont(self.defaultFontBold)
        grJobControl.setGeometry(1110, 10, 371, 301)

        self.btnTestStartTime = QPushButton("Start Time", grJobControl)
        self.btnTestStartTime.setGeometry(10, 40, 111, 41)

        self.btnTestStopTime = QPushButton("Stop Time", grJobControl)
        self.btnTestStopTime.setGeometry(10, 90, 111, 41)

        self.txtTime = QLineEdit("0.00", grJobControl)
        self.txtTime.setAlignment(Qt.AlignCenter)
        self.txtTime.setGeometry(10, 140, 111, 41)

        self.btnMoveJob = QPushButton("MOVE JOB", grJobControl)
        self.btnMoveJob.setGeometry(10, 250, 111, 41)

        self.btnTestPositionDetect = QPushButton("Position Detect", grJobControl)
        self.btnTestPositionDetect.setGeometry(130, 40, 111, 41)

        self.btnTestMovePosition = QPushButton("Move Position", grJobControl)
        self.btnTestMovePosition.setGeometry(130, 90, 111, 41)

        self.txtByte5 = QLineEdit("Byte 5 = None", grJobControl)
        self.txtByte5.setAlignment(Qt.AlignCenter)
        self.txtByte5.setGeometry(130, 140, 111, 41)

        self.btnExitJob = QPushButton("EXIT JOB", grJobControl)
        self.btnExitJob.setGeometry(130, 250, 111, 41)

        self.lbOjectStatus = QLabel("Object: False", grJobControl)
        self.lbOjectStatus.setAlignment(Qt.AlignCenter)
        self.lbOjectStatus.setGeometry(250, 40, 111, 41)

        self.lbGripperStatus = QLabel("Gripper: None", grJobControl)
        self.lbGripperStatus.setAlignment(Qt.AlignCenter)
        self.lbGripperStatus.setGeometry(250, 90, 111, 41)

        self.btnTestRunJob = QPushButton("Run Test Job", grJobControl)
        self.btnTestRunJob.setGeometry(250, 140, 111, 41)

        self.btnCaptureBackground = QPushButton("BACKGROUND", grJobControl)
        self.btnCaptureBackground.setGeometry(250, 250, 111, 41)

    def initCameraGroup(self, mw):
        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grCamera = QGroupBox(mw)
        grCamera.setTitle("CAMERA")
        grCamera.setFont(self.defaultFontBold)
        grCamera.setGeometry(610, 320, 871, 531)

        self.btnOpenCloseCamera = QPushButton("Open Camera", grCamera)
        self.btnOpenCloseCamera.setFont(self.defaultFontRegular)
        self.btnOpenCloseCamera.setGeometry(30, 30, 151, 41)

        self.btnRGBCamera = QPushButton("RGB Frame", grCamera)
        self.btnRGBCamera.setFont(self.defaultFontRegular)
        self.btnRGBCamera.setGeometry(30, 80, 151, 41)

        self.btnBinaryCamera = QPushButton("Binary Frame", grCamera)
        self.btnBinaryCamera.setFont(self.defaultFontRegular)
        self.btnBinaryCamera.setGeometry(30, 130, 151, 41)

        self.btnDepthCamera = QPushButton("Depth Frame", grCamera)
        self.btnDepthCamera.setFont(self.defaultFontRegular)
        self.btnDepthCamera.setGeometry(30, 180, 151, 41)

        self.btnCaptureAruco = QPushButton("Capture ArUco", grCamera)
        self.btnCaptureAruco.setFont(self.defaultFontRegular)
        self.btnCaptureAruco.setGeometry(30, 230, 151, 41)

        self.btnCaptureObject = QPushButton("Capture Object", grCamera)
        self.btnCaptureObject.setFont(self.defaultFontRegular)
        self.btnCaptureObject.setGeometry(30, 280, 151, 41)

        self.txtImageCount = QLabel("Image_(0)", grCamera)
        self.txtImageCount.setFont(fontBold)
        self.txtImageCount.setAlignment(Qt.AlignCenter)
        self.txtImageCount.setGeometry(30, 330, 151, 31)

        self.btnYoloDetect = QPushButton("YOLO Detect ON", grCamera)
        self.btnYoloDetect.setFont(self.defaultFontRegular)
        self.btnYoloDetect.setGeometry(30, 370, 151, 41)

        self.btnColorDetect = QPushButton("COLOR Detect ON", grCamera)
        self.btnColorDetect.setFont(self.defaultFontRegular)
        self.btnColorDetect.setGeometry(30, 420, 151, 41)

        self.btnArucoDetect = QPushButton("ArUco Detect ON", grCamera)
        self.btnArucoDetect.setFont(self.defaultFontRegular)
        self.btnArucoDetect.setGeometry(30, 470, 151, 41)

        self.frameDisplayCamera = QLabel(grCamera)
        self.frameDisplayCamera.setGeometry(210, 30, 640, 480)
        self.frameDisplayCamera.setFrameShape(QFrame.Box)


class SwitchButton(QPushButton):
    signalSwitchToggled = pyqtSignal(bool)

    def __init__(self, mw, grBox, x, y):
        super().__init__()
        self.mainWindow = mw
        self.status = False
        self.btnBackground = QPushButton(grBox)
        self.btnBackground.setGeometry(x, y, 60, 30)
        self.btnBackground.setStyleSheet(""" 
            QPushButton {
                background-color: lightgray;
                border-radius: 15px;
            }""")
        self.btnCircle = QPushButton(self.btnBackground)
        self.btnCircle.setGeometry(0, 0, 26, 30)
        self.btnCircle.setStyleSheet("""
            QPushButton {
                background-color: white;
                border-radius: 13px;
            }""")
        self.animation = QPropertyAnimation(self.btnCircle, b'geometry')
        self.btnCircle.clicked.connect(self.animationToggled)

    def animationToggled(self):
        if self.status:
            self.status = False
            self.animation.setStartValue(QRect(self.btnCircle.pos().x(), self.btnCircle.pos().y(),
                                               self.btnCircle.width(), self.btnCircle.height()))
            self.animation.setEndValue(QRect(self.btnCircle.pos().x() - (self.btnBackground.width() - self.btnCircle.width()),
                                             self.btnCircle.pos().y(), self.btnCircle.width(), self.btnCircle.height()))
            self.btnBackground.setStyleSheet(""" 
                QPushButton {
                    background-color: lightgray;
                    border-radius: 15px;
                }""")
        else:
            self.status = True
            self.animation.setStartValue(QRect(self.btnCircle.pos().x(), self.btnCircle.pos().y(),
                                               self.btnCircle.width(), self.btnCircle.height()))
            self.animation.setEndValue(QRect(self.btnCircle.pos().x() + (self.btnBackground.width() - self.btnCircle.width()),
                                             self.btnCircle.pos().y(), self.btnCircle.width(), self.btnCircle.height()))
            self.btnBackground.setStyleSheet(""" 
                QPushButton {
                    background-color: green;
                    border-radius: 15px;
                }""")

        self.animation.setDuration(200)
        self.animation.start()
        self.signalSwitchToggled.emit(self.status)


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = QMainWindow()
#     uic = MainWindowUI()
#     uic.initUI(window)
#     window.show()
#     sys.exit(app.exec_())
