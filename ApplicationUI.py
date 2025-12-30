from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QPushButton, QTextEdit, QSlider, QLabel, QLineEdit
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox
from PyQt5.QtWidgets import QMessageBox, QSizePolicy, QFrame
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QUrl, QSize, QRect, QCoreApplication, QPropertyAnimation, pyqtSignal


class MainWindowUI(object):
    def initUI(self, mainWindow):
        mainWindow.setObjectName("MainWindow")
        mainWindow.setWindowTitle("Robot Application")
        centralWidget = QWidget()
        mainWindow.setCentralWidget(centralWidget)

        self.defaultFontBold = QFont("Time New Roman", 10)
        self.defaultFontBold.setBold(True)
        self.defaultFontRegular = QFont("Time New Roman", 10)
        self.defaultFontRegular.setBold(False)

        self.leftMainLayout = QVBoxLayout()
        self.rightMainLayout = QVBoxLayout()
        self.rightTopLayout = QHBoxLayout()
        self.rightMainLayout.addLayout(self.rightTopLayout)

        self.initConnectionGroup(mainWindow)
        self.initBasicControlGroup(mainWindow)
        self.initIncrementalControlGroup(mainWindow)
        self.initPositionGroup(mainWindow)
        self.initJobControlGroup(mainWindow)
        self.initCameraGroup(mainWindow)

        self.mainLayout = QHBoxLayout()
        self.mainLayout.addLayout(self.leftMainLayout)
        self.mainLayout.addLayout(self.rightMainLayout)
        centralWidget.setLayout(self.mainLayout)

    def initConnectionGroup(self, mw):
        fontBold = QFont("Time New Roman", 12)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 12)
        fontRegular.setBold(False)

        grConnection = QGroupBox()
        grConnection.setTitle("CONNECTION")
        grConnection.setFont(self.defaultFontBold)

        layoutConnection = QHBoxLayout(grConnection)

        # Group Robot inside Group Connection
        grRobot = QGroupBox(grConnection)
        grRobot.setTitle("ROBOT")

        layoutRobot = QVBoxLayout(grRobot)
        layoutTextRobot = QGridLayout()
        layoutTextRobot.setHorizontalSpacing(20)
        layoutDeviceRobot = QGridLayout()

        lbIPRobot = QLabel("IP Address", grRobot)
        lbIPRobot.setFont(self.defaultFontRegular)
        lbIPRobot.setAlignment(Qt.AlignLeft)

        self.txtIPRobot = QLineEdit("192.168.1.16", grRobot)
        self.txtIPRobot.setAlignment(Qt.AlignLeft)

        lbPortRobot = QLabel("Port", grRobot)
        lbPortRobot.setFont(self.defaultFontRegular)
        lbPortRobot.setAlignment(Qt.AlignLeft)

        self.txtPortRobot = QLineEdit("10040", grRobot)
        self.txtPortRobot.setAlignment(Qt.AlignLeft)

        layoutTextRobot.addWidget(lbIPRobot, 0, 0)
        layoutTextRobot.addWidget(self.txtIPRobot, 0, 1)
        layoutTextRobot.addWidget(lbPortRobot, 1, 0)
        layoutTextRobot.addWidget(self.txtPortRobot, 1, 1)

        self.btnRobotConDis = QPushButton("Connect", grRobot)
        self.btnRobotConDis.setFont(fontBold)
        self.btnRobotConDis.setFixedSize(250, 45)

        lbDeviceRobotStatus = QLabel("Device Robot:", grRobot)
        lbDeviceRobotStatus.setFont(self.defaultFontRegular)
        lbDeviceRobotStatus.setAlignment(Qt.AlignLeft)
        lbDeviceRobotStatus.setFixedSize(130, 30)

        self.btnDeviceRobot = SwitchButton()

        lbRobotConnectStatus = QLabel("Connection Robot:", grRobot)
        lbRobotConnectStatus.setFont(self.defaultFontRegular)
        lbRobotConnectStatus.setAlignment(Qt.AlignLeft)
        lbRobotConnectStatus.setFixedSize(130, 30)

        self.txtRobotConnectStatus = QLabel("Disconnected", grRobot)
        self.txtRobotConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtRobotConnectStatus.setFixedSize(130, 30)

        layoutDeviceRobot.addWidget(lbDeviceRobotStatus, 0, 0)
        layoutDeviceRobot.addWidget(self.btnDeviceRobot, 0, 1)
        layoutDeviceRobot.addWidget(lbRobotConnectStatus, 1, 0)
        layoutDeviceRobot.addWidget(self.txtRobotConnectStatus, 1, 1)

        layoutRobot.addLayout(layoutTextRobot)
        layoutRobot.addSpacing(5)
        layoutRobot.addWidget(self.btnRobotConDis, alignment=Qt.AlignHCenter)
        layoutRobot.addSpacing(5)
        layoutRobot.addLayout(layoutDeviceRobot)
        grRobot.setLayout(layoutRobot)

        # Group Gripper inside Group Connection
        grGripper = QGroupBox(grConnection)
        grGripper.setTitle("GRIPPER")

        layoutGripper = QVBoxLayout(grGripper)
        layoutTextGripper = QGridLayout()
        layoutTextGripper.setHorizontalSpacing(20)
        layoutDeviceGripper = QGridLayout()

        lbPortGripper = QLabel("Port", grGripper)
        lbPortGripper.setFont(self.defaultFontRegular)
        lbPortGripper.setAlignment(Qt.AlignLeft)

        self.txtPortGripper = QLineEdit("COM4", grGripper)
        self.txtPortGripper.setAlignment(Qt.AlignLeft)

        lbBaudGripper = QLabel("Baudrate", grGripper)
        lbBaudGripper.setFont(self.defaultFontRegular)
        lbBaudGripper.setAlignment(Qt.AlignLeft)

        self.txtBaudGripper = QLineEdit("115200", grGripper)
        self.txtBaudGripper.setAlignment(Qt.AlignLeft)

        layoutTextGripper.addWidget(lbPortGripper, 0, 0)
        layoutTextGripper.addWidget(self.txtPortGripper, 0, 1)
        layoutTextGripper.addWidget(lbBaudGripper, 1, 0)
        layoutTextGripper.addWidget(self.txtBaudGripper, 1, 1)

        self.btnGripperConDis = QPushButton("Connect", grGripper)
        self.btnGripperConDis.setFont(fontBold)
        self.btnGripperConDis.setFixedSize(250, 45)

        lbDeviceIOStatus = QLabel("Device I/O:", grGripper)
        lbDeviceIOStatus.setFont(self.defaultFontRegular)
        lbDeviceIOStatus.setAlignment(Qt.AlignLeft)
        lbDeviceIOStatus.setFixedSize(130, 30)

        self.btnDeviceIO = SwitchButton()

        lbIOConnectStatus = QLabel("Connection I/O:", grGripper)
        lbIOConnectStatus.setFont(self.defaultFontRegular)
        lbIOConnectStatus.setAlignment(Qt.AlignLeft)
        lbIOConnectStatus.setFixedSize(130, 30)

        self.txtIOConnectStatus = QLabel("Disconnected", grGripper)
        self.txtIOConnectStatus.setFont(self.defaultFontBold)
        self.txtIOConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtIOConnectStatus.setFixedSize(130, 30)

        layoutDeviceGripper.addWidget(lbDeviceIOStatus, 0, 0)
        layoutDeviceGripper.addWidget(self.btnDeviceIO, 0, 1)
        layoutDeviceGripper.addWidget(lbIOConnectStatus, 1, 0)
        layoutDeviceGripper.addWidget(self.txtIOConnectStatus, 1, 1)

        layoutGripper.addLayout(layoutTextGripper)
        layoutGripper.addSpacing(5)
        layoutGripper.addWidget(self.btnGripperConDis, alignment=Qt.AlignHCenter)
        layoutGripper.addSpacing(5)
        layoutGripper.addLayout(layoutDeviceGripper)
        grGripper.setLayout(layoutGripper)

        layoutConnection.addWidget(grRobot)
        layoutConnection.addWidget(grGripper)
        grConnection.setLayout(layoutConnection)
        self.leftMainLayout.addWidget(grConnection)

    def initBasicControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 12)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 12)
        fontRegular.setBold(False)

        grBasicControl = QGroupBox()
        grBasicControl.setTitle("BASIC CONTROL")
        grBasicControl.setFont(self.defaultFontBold)

        layoutBasic = QGridLayout(grBasicControl)

        self.btnAutoMode = QPushButton("Auto", grBasicControl)
        self.btnAutoMode.setFixedSize(140, 50)

        self.btnManualMode = QPushButton("Manual", grBasicControl)
        self.btnManualMode.setFixedSize(160, 50)

        self.btnServo = QPushButton("Servo On", grBasicControl)
        self.btnServo.setFixedSize(180, 50)

        self.btnHome = QPushButton("Go Home", grBasicControl)
        self.btnHome.setFixedSize(180, 50)

        self.btnSerialSend = QPushButton("Send Data Serial", grBasicControl)
        self.btnSerialSend.setFixedSize(210, 50)

        self.txtSerialValue = QLineEdit("80", grBasicControl)
        self.txtSerialValue.setAlignment(Qt.AlignCenter)
        self.txtSerialValue.setFont(fontBold)
        self.txtSerialValue.setFixedSize(210, 50)

        lbMode = QLabel("MODE", grBasicControl)
        lbMode.setAlignment(Qt.AlignCenter)
        lbMode.setFont(fontRegular)
        lbMode.setFixedSize(140, 50)

        self.txtModeValue = QLabel("NONE", grBasicControl)
        self.txtModeValue.setFont(fontRegular)
        self.txtModeValue.setAlignment(Qt.AlignCenter)
        self.txtModeValue.setFixedSize(160, 50)

        layoutBasic.addWidget(self.btnAutoMode, 0, 0)
        layoutBasic.addWidget(self.btnManualMode, 0, 1)
        layoutBasic.addWidget(self.btnServo, 0, 2)
        layoutBasic.addWidget(self.txtSerialValue, 0, 3)
        layoutBasic.addWidget(lbMode, 1, 0)
        layoutBasic.addWidget(self.txtModeValue, 1, 1)
        layoutBasic.addWidget(self.btnHome, 1, 2)
        layoutBasic.addWidget(self.btnSerialSend, 1, 3)
        grBasicControl.setLayout(layoutBasic)
        self.leftMainLayout.addWidget(grBasicControl)

    def initIncrementalControlGroup(self, mw):
        WIDTH_SIZE_DEFAULT = 100
        HEIGHT_SIZE_DEFAULT = 50
        fontBold = QFont("Time New Roman", 12)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 12)
        fontRegular.setBold(False)

        grIncremental = QGroupBox()
        grIncremental.setTitle("INCREMENTAL POSITION")
        grIncremental.setFont(self.defaultFontBold)
        grIncremental.setAlignment(Qt.AlignLeft)

        layoutIncremental = QHBoxLayout(grIncremental)
        layoutIncremental.setSpacing(50)
        layoutLeft = QGridLayout()
        layoutLeft.setVerticalSpacing(30)
        layoutLeft.setHorizontalSpacing(10)
        layoutRight = QGridLayout()
        layoutRight.setVerticalSpacing(10)
        layoutRight.setHorizontalSpacing(5)

        lbX = QLabel("X", grIncremental)
        lbX.setFont(self.defaultFontRegular)
        lbX.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbX.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnXInc = QPushButton("+", grIncremental)
        self.btnXInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnXDec = QPushButton("-", grIncremental)
        self.btnXDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        lbY = QLabel("Y", grIncremental)
        lbY.setFont(self.defaultFontRegular)
        lbY.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbY.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnYInc = QPushButton("+", grIncremental)
        self.btnYInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnYDec = QPushButton("-", grIncremental)
        self.btnYDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        lbZ = QLabel("Z", grIncremental)
        lbZ.setFont(self.defaultFontRegular)
        lbZ.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbZ.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnZInc = QPushButton("+", grIncremental)
        self.btnZInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnZDec = QPushButton("-", grIncremental)
        self.btnZDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        lbRoll = QLabel("Roll", grIncremental)
        lbRoll.setFont(self.defaultFontRegular)
        lbRoll.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbRoll.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnRollInc = QPushButton("+", grIncremental)
        self.btnRollInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnRollDec = QPushButton("-", grIncremental)
        self.btnRollDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        lbPitch = QLabel("Pitch", grIncremental)
        lbPitch.setFont(self.defaultFontRegular)
        lbPitch.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbPitch.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnPitchInc = QPushButton("+", grIncremental)
        self.btnPitchInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnPitchDec = QPushButton("-", grIncremental)
        self.btnPitchDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        lbYaw = QLabel("Yaw", grIncremental)
        lbYaw.setFont(self.defaultFontRegular)
        lbYaw.setAlignment(Qt.AlignCenter | Qt.AlignHCenter | Qt.AlignVCenter)
        lbYaw.setFixedHeight(HEIGHT_SIZE_DEFAULT)

        self.btnYawInc = QPushButton("+", grIncremental)
        self.btnYawInc.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)
        self.btnYawDec = QPushButton("-", grIncremental)
        self.btnYawDec.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        layoutLeft.addWidget(lbX, 0, 0)
        layoutLeft.addWidget(self.btnXInc, 0, 1)
        layoutLeft.addWidget(self.btnXDec, 0, 2)
        layoutLeft.addWidget(lbY, 1, 0)
        layoutLeft.addWidget(self.btnYInc, 1, 1)
        layoutLeft.addWidget(self.btnYDec, 1, 2)
        layoutLeft.addWidget(lbZ, 2, 0)
        layoutLeft.addWidget(self.btnZInc, 2, 1)
        layoutLeft.addWidget(self.btnZDec, 2, 2)
        layoutLeft.addWidget(lbRoll, 3, 0)
        layoutLeft.addWidget(self.btnRollInc, 3, 1)
        layoutLeft.addWidget(self.btnRollDec, 3, 2)
        layoutLeft.addWidget(lbPitch, 4, 0)
        layoutLeft.addWidget(self.btnPitchInc, 4, 1)
        layoutLeft.addWidget(self.btnPitchDec, 4, 2)
        layoutLeft.addWidget(lbYaw, 5, 0)
        layoutLeft.addWidget(self.btnYawInc, 5, 1)
        layoutLeft.addWidget(self.btnYawDec, 5, 2)

        lbDis = QLabel("DISTANCE", grIncremental)
        lbDis.setFont(fontBold)

        self.sliderDistanceMM = QSlider(grIncremental)
        self.sliderDistanceMM.setMinimum(0)
        self.sliderDistanceMM.setMaximum(10000)
        self.sliderDistanceMM.setSingleStep(1)
        self.sliderDistanceMM.setOrientation(Qt.Horizontal)

        self.txtDisMM = QLabel("0", grIncremental)
        self.txtDisMM.setAlignment(Qt.AlignCenter)
        self.txtDisMM.setFont(self.defaultFontRegular)

        lbMM = QLabel("mm", grIncremental)
        lbMM.setAlignment(Qt.AlignCenter)
        lbMM.setFont(self.defaultFontRegular)

        self.sliderDistanceDEG = QSlider(grIncremental)
        self.sliderDistanceDEG.setMinimum(0)
        self.sliderDistanceDEG.setMaximum(100000)
        self.sliderDistanceDEG.setSingleStep(1)
        self.sliderDistanceDEG.setOrientation(Qt.Horizontal)

        self.txtDisDEG = QLabel("0", grIncremental)
        self.txtDisDEG.setAlignment(Qt.AlignCenter)
        self.txtDisDEG.setFont(self.defaultFontRegular)

        lbDEG = QLabel("deg", grIncremental)
        lbDEG.setAlignment(Qt.AlignCenter)
        lbDEG.setFont(self.defaultFontRegular)

        lbSpeManual = QLabel("SPEED MANUAL", grIncremental)
        lbSpeManual.setFont(fontBold)

        self.sliderSpeedMM = QSlider(grIncremental)
        self.sliderSpeedMM.setMinimum(0)
        self.sliderSpeedMM.setMaximum(300)
        self.sliderSpeedMM.setSingleStep(1)
        self.sliderSpeedMM.setOrientation(Qt.Horizontal)

        self.txtSpeMM = QLabel("0", grIncremental)
        self.txtSpeMM.setAlignment(Qt.AlignCenter)
        self.txtSpeMM.setFont(self.defaultFontRegular)

        lbMMs = QLabel("mm/s", grIncremental)
        lbMMs.setAlignment(Qt.AlignCenter)
        lbMMs.setFont(self.defaultFontRegular)

        self.sliderSpeedDEG = QSlider(grIncremental)
        self.sliderSpeedDEG.setMinimum(0)
        self.sliderSpeedDEG.setMaximum(1000)
        self.sliderSpeedDEG.setSingleStep(1)
        self.sliderSpeedDEG.setOrientation(Qt.Horizontal)

        self.txtSpeDEG = QLabel("0", grIncremental)
        self.txtSpeDEG.setAlignment(Qt.AlignCenter)
        self.txtSpeDEG.setFont(self.defaultFontRegular)

        lbDEGs = QLabel("deg/s", grIncremental)
        lbDEGs.setAlignment(Qt.AlignCenter)
        lbDEGs.setFont(self.defaultFontRegular)

        lbSpeAuto = QLabel("SPEED AUTO", grIncremental)
        lbSpeAuto.setFont(fontBold)

        self.sliderSpeedAuto = QSlider(grIncremental)
        self.sliderSpeedAuto.setMinimum(0)
        self.sliderSpeedAuto.setMaximum(300)
        self.sliderSpeedAuto.setSingleStep(1)
        self.sliderSpeedAuto.setOrientation(Qt.Horizontal)

        self.txtSpeAuto = QLabel("0", grIncremental)
        self.txtSpeAuto.setAlignment(Qt.AlignCenter)
        self.txtSpeAuto.setFont(self.defaultFontRegular)

        lbMMsAuto = QLabel("mm/s", grIncremental)
        lbMMsAuto.setAlignment(Qt.AlignCenter)
        lbMMsAuto.setFont(self.defaultFontRegular)

        layoutRight.addWidget(lbDis, 0, 0, alignment=Qt.AlignTop)
        layoutRight.addWidget(self.sliderDistanceMM, 1, 0, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.txtDisMM, 1, 1, alignment=Qt.AlignCenter)
        layoutRight.addWidget(lbMM, 1, 2, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.sliderDistanceDEG, 2, 0, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.txtDisDEG, 2, 1, alignment=Qt.AlignCenter)
        layoutRight.addWidget(lbDEG, 2, 2, alignment=Qt.AlignCenter)

        layoutRight.addWidget(lbSpeManual, 3, 0, alignment=Qt.AlignTop)
        layoutRight.addWidget(self.sliderSpeedMM, 4, 0, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.txtSpeMM, 4, 1, alignment=Qt.AlignCenter)
        layoutRight.addWidget(lbMMs, 4, 2, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.sliderSpeedDEG, 5, 0, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.txtSpeDEG, 5, 1, alignment=Qt.AlignCenter)
        layoutRight.addWidget(lbDEGs, 5, 2, alignment=Qt.AlignCenter)

        layoutRight.addWidget(lbSpeAuto, 6, 0)
        layoutRight.addWidget(self.sliderSpeedAuto, 7, 0, alignment=Qt.AlignCenter)
        layoutRight.addWidget(self.txtSpeAuto, 7, 1, alignment=Qt.AlignCenter)
        layoutRight.addWidget(lbMMsAuto, 7, 2, alignment=Qt.AlignCenter)

        layoutIncremental.addLayout(layoutLeft)
        layoutIncremental.addLayout(layoutRight)
        grIncremental.setLayout(layoutIncremental)
        self.leftMainLayout.addWidget(grIncremental)

    def initPositionGroup(self, mw):
        WIDTH_SIZE_DEFAULT = 180
        HEIGHT_SIZE_DEFAULT = 35

        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grPosition = QGroupBox()
        grPosition.setTitle("GET POSITION")
        grPosition.setFont(self.defaultFontBold)

        layoutPosition = QVBoxLayout(grPosition)

        grPulse = QGroupBox("PULSE", grPosition)
        grPulse.setFont(self.defaultFontBold)
        grPulse.setAlignment(Qt.AlignCenter)

        layoutPulse = QGridLayout(grPulse)

        lbS = QLabel("S", grPulse)
        lbS.setFont(self.defaultFontRegular)

        lbL = QLabel("L", grPulse)
        lbL.setFont(self.defaultFontRegular)

        lbU = QLabel("U", grPulse)
        lbU.setFont(self.defaultFontRegular)

        lbR = QLabel("R", grPulse)
        lbR.setFont(self.defaultFontRegular)

        lbB = QLabel("B", grPulse)
        lbB.setFont(self.defaultFontRegular)

        lbT = QLabel("T", grPulse)
        lbT.setFont(self.defaultFontRegular)

        self.txtS = QLineEdit(grPulse)
        self.txtS.setAlignment(Qt.AlignCenter)
        self.txtS.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtL = QLineEdit(grPulse)
        self.txtL.setAlignment(Qt.AlignCenter)
        self.txtL.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtU = QLineEdit(grPulse)
        self.txtU.setAlignment(Qt.AlignCenter)
        self.txtU.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtR = QLineEdit(grPulse)
        self.txtR.setAlignment(Qt.AlignCenter)
        self.txtR.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtB = QLineEdit(grPulse)
        self.txtB.setAlignment(Qt.AlignCenter)
        self.txtB.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtT = QLineEdit(grPulse)
        self.txtT.setAlignment(Qt.AlignCenter)
        self.txtT.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        for i in range(6):
            lbDeg = QLabel("deg", grPulse)
            lbDeg.setFont(self.defaultFontRegular)
            if i == 0:
                layoutPulse.addWidget(lbS, i, 0)
                layoutPulse.addWidget(self.txtS, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)
            elif i == 1:
                layoutPulse.addWidget(lbL, i, 0)
                layoutPulse.addWidget(self.txtL, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)
            elif i == 2:
                layoutPulse.addWidget(lbU, i, 0)
                layoutPulse.addWidget(self.txtU, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)
            elif i == 3:
                layoutPulse.addWidget(lbR, i, 0)
                layoutPulse.addWidget(self.txtR, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)
            elif i == 4:
                layoutPulse.addWidget(lbB, i, 0)
                layoutPulse.addWidget(self.txtB, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)
            elif i == 5:
                layoutPulse.addWidget(lbT, i, 0)
                layoutPulse.addWidget(self.txtT, i, 1)
                layoutPulse.addWidget(lbDeg, i, 2)

        grPulse.setLayout(layoutPulse)

        grCartesian = QGroupBox("CARTESIAN", grPosition)
        grCartesian.setFont(self.defaultFontBold)
        grCartesian.setAlignment(Qt.AlignCenter)

        layoutCartesian = QGridLayout(grCartesian)

        lbX = QLabel("X", grCartesian)
        lbX.setFont(self.defaultFontRegular)
        lbX.setAlignment(Qt.AlignCenter)

        lbY = QLabel("Y", grCartesian)
        lbY.setFont(self.defaultFontRegular)
        lbY.setAlignment(Qt.AlignCenter)

        lbZ = QLabel("Z", grCartesian)
        lbZ.setFont(self.defaultFontRegular)
        lbZ.setAlignment(Qt.AlignCenter)

        lbRoll = QLabel("Roll", grCartesian)
        lbRoll.setFont(self.defaultFontRegular)
        lbRoll.setAlignment(Qt.AlignCenter)

        lbPitch = QLabel("Pitch", grCartesian)
        lbPitch.setFont(self.defaultFontRegular)
        lbPitch.setAlignment(Qt.AlignCenter)

        lbYaw = QLabel("Yaw", grCartesian)
        lbYaw.setFont(self.defaultFontRegular)
        lbYaw.setAlignment(Qt.AlignCenter)

        self.txtX = QLineEdit(grCartesian)
        self.txtX.setAlignment(Qt.AlignCenter)
        self.txtX.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtY = QLineEdit(grCartesian)
        self.txtY.setAlignment(Qt.AlignCenter)
        self.txtY.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtZ = QLineEdit(grCartesian)
        self.txtZ.setAlignment(Qt.AlignCenter)
        self.txtZ.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtRoll = QLineEdit(grCartesian)
        self.txtRoll.setAlignment(Qt.AlignCenter)
        self.txtRoll.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtPitch = QLineEdit(grCartesian)
        self.txtPitch.setAlignment(Qt.AlignCenter)
        self.txtPitch.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtYaw = QLineEdit(grCartesian)
        self.txtYaw.setAlignment(Qt.AlignCenter)
        self.txtYaw.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        for i in range(6):
            if i < 3:
                lbMM = QLabel("mm", grCartesian)
                lbMM.setFont(self.defaultFontRegular)
                if i == 0:
                    layoutCartesian.addWidget(lbX, i, 0)
                    layoutCartesian.addWidget(self.txtX, i, 1)
                    layoutCartesian.addWidget(lbMM, i, 2)
                elif i == 1:
                    layoutCartesian.addWidget(lbY, i, 0)
                    layoutCartesian.addWidget(self.txtY, i, 1)
                    layoutCartesian.addWidget(lbMM, i, 2)
                elif i == 2:
                    layoutCartesian.addWidget(lbZ, i, 0)
                    layoutCartesian.addWidget(self.txtZ, i, 1)
                    layoutCartesian.addWidget(lbMM, i, 2)
            else:
                lbDeg = QLabel("deg", grCartesian)
                lbDeg.setFont(self.defaultFontRegular)
                if i == 3:
                    layoutCartesian.addWidget(lbRoll, i, 0)
                    layoutCartesian.addWidget(self.txtRoll, i, 1)
                    layoutCartesian.addWidget(lbDeg, i, 2)
                elif i == 4:
                    layoutCartesian.addWidget(lbPitch, i, 0)
                    layoutCartesian.addWidget(self.txtPitch, i, 1)
                    layoutCartesian.addWidget(lbDeg, i, 2)
                elif i == 5:
                    layoutCartesian.addWidget(lbYaw, i, 0)
                    layoutCartesian.addWidget(self.txtYaw, i, 1)
                    layoutCartesian.addWidget(lbDeg, i, 2)

        grCartesian.setLayout(layoutCartesian)

        self.btnSetPosition = QPushButton("Set Position", grPosition)
        self.btnSetPosition.setFont(fontBold)
        self.btnSetPosition.setFixedHeight(50)

        self.btnGetPosition = QPushButton("Get Position", grPosition)
        self.btnGetPosition.setFont(fontBold)
        self.btnGetPosition.setFixedHeight(50)

        layoutTypePosition = QGridLayout()
        layoutTypePosition.addWidget(grPulse, 0, 0)
        layoutTypePosition.addWidget(grCartesian, 0, 1)
        layoutTypePosition.addWidget(self.btnSetPosition, 1, 0)
        layoutTypePosition.addWidget(self.btnGetPosition, 1, 1)

        layoutPosition.addLayout(layoutTypePosition)
        grPosition.setLayout(layoutPosition)

        self.rightTopLayout.addWidget(grPosition)

    def initJobControlGroup(self, mw):
        WIDTH_SIZE_DEFAULT = 170
        HEIGHT_SIZE_DEFAULT = 50

        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grJobControl = QGroupBox()
        grJobControl.setTitle("JOB CONTROL")
        grJobControl.setFont(self.defaultFontBold)

        layoutJobControl = QVBoxLayout(grJobControl)

        self.btnTestStartTime = QPushButton("Start Time", grJobControl)
        self.btnTestStartTime.setFont(self.defaultFontRegular)
        self.btnTestStartTime.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnTestStopTime = QPushButton("Stop Time", grJobControl)
        self.btnTestStopTime.setFont(self.defaultFontRegular)
        self.btnTestStopTime.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtTime = QLineEdit("0.00", grJobControl)
        self.txtTime.setAlignment(Qt.AlignCenter)
        self.txtTime.setFont(self.defaultFontRegular)
        self.txtTime.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnMoveJob = QPushButton("MOVE JOB", grJobControl)
        self.btnMoveJob.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnTestPositionDetect = QPushButton("Position Detect", grJobControl)
        self.btnTestPositionDetect.setFont(self.defaultFontRegular)
        self.btnTestPositionDetect.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnTestMovePosition = QPushButton("Move Position", grJobControl)
        self.btnTestMovePosition.setFont(self.defaultFontRegular)
        self.btnTestMovePosition.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtByte5 = QLineEdit("Byte 5 = None", grJobControl)
        self.txtByte5.setAlignment(Qt.AlignCenter)
        self.txtByte5.setFont(self.defaultFontRegular)
        self.txtByte5.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnExitJob = QPushButton("EXIT JOB", grJobControl)
        self.btnExitJob.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.lbOjectStatus = QLabel("Object: False", grJobControl)
        self.lbOjectStatus.setAlignment(Qt.AlignCenter)
        self.lbOjectStatus.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.lbGripperStatus = QLabel("Gripper: None", grJobControl)
        self.lbGripperStatus.setAlignment(Qt.AlignCenter)
        self.lbGripperStatus.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnTestRunJob = QPushButton("Run Test Job", grJobControl)
        self.btnTestRunJob.setFont(self.defaultFontRegular)
        self.btnTestRunJob.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnCaptureBackground = QPushButton("BACKGROUND", grJobControl)
        self.btnCaptureBackground.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        layoutTest = QGridLayout()
        layoutTest.addWidget(self.btnTestStartTime, 0, 0)
        layoutTest.addWidget(self.btnTestPositionDetect, 0, 1)
        layoutTest.addWidget(self.lbOjectStatus, 0, 2)
        layoutTest.addWidget(self.btnTestStopTime, 1, 0)
        layoutTest.addWidget(self.btnTestMovePosition, 1, 1)
        layoutTest.addWidget(self.lbGripperStatus, 1, 2)
        layoutTest.addWidget(self.txtTime, 2, 0)
        layoutTest.addWidget(self.txtByte5, 2, 1)
        layoutTest.addWidget(self.btnTestRunJob, 2, 2)

        layoutProcess = QGridLayout()
        layoutProcess.addWidget(self.btnMoveJob, 0, 0)
        layoutProcess.addWidget(self.btnExitJob, 0, 1)
        layoutProcess.addWidget(self.btnCaptureBackground, 0, 2)

        layoutJobControl.addLayout(layoutTest, Qt.AlignTop)
        layoutJobControl.addSpacing(150)
        layoutJobControl.addLayout(layoutProcess, Qt.AlignBottom)
        grJobControl.setLayout(layoutJobControl)
        self.rightTopLayout.addWidget(grJobControl)

    def initCameraGroup(self, mw):
        WIDTH_SIZE_DEFAULT = 170
        HEIGHT_SIZE_DEFAULT = 50

        fontBold = QFont("Time New Roman", 10)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 10)
        fontRegular.setBold(False)

        grCamera = QGroupBox()
        grCamera.setTitle("CAMERA")
        grCamera.setFont(self.defaultFontBold)

        layoutCamera = QHBoxLayout()
        layoutLeftButton = QVBoxLayout()
        layoutLeftButton.setSpacing(1)
        layoutLeftButton.setContentsMargins(0, 0, 0, 0)
        layoutRightButton = QVBoxLayout()
        layoutRightButton.setSpacing(1)
        layoutRightButton.setContentsMargins(0, 0, 0, 0)

        self.btnOpenCloseCamera = QPushButton("Open Camera", grCamera)
        self.btnOpenCloseCamera.setFont(self.defaultFontRegular)
        self.btnOpenCloseCamera.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnRGBCamera = QPushButton("RGB Frame", grCamera)
        self.btnRGBCamera.setFont(self.defaultFontRegular)
        self.btnRGBCamera.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnBinaryCamera = QPushButton("Binary Frame", grCamera)
        self.btnBinaryCamera.setFont(self.defaultFontRegular)
        self.btnBinaryCamera.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnDepthCamera = QPushButton("Depth Frame", grCamera)
        self.btnDepthCamera.setFont(self.defaultFontRegular)
        self.btnDepthCamera.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnCaptureAruco = QPushButton("Capture ArUco", grCamera)
        self.btnCaptureAruco.setFont(self.defaultFontRegular)
        self.btnCaptureAruco.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnCaptureObject = QPushButton("Capture Object", grCamera)
        self.btnCaptureObject.setFont(self.defaultFontRegular)
        self.btnCaptureObject.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.txtImageCount = QLabel("Image_(0)", grCamera)
        self.txtImageCount.setFont(fontBold)
        self.txtImageCount.setAlignment(Qt.AlignCenter)
        self.txtImageCount.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnYoloDetect = QPushButton("YOLO Detect ON", grCamera)
        self.btnYoloDetect.setFont(self.defaultFontRegular)
        self.btnYoloDetect.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnColorDetect = QPushButton("COLOR Detect ON", grCamera)
        self.btnColorDetect.setFont(self.defaultFontRegular)
        self.btnColorDetect.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        self.btnArucoDetect = QPushButton("ArUco Detect ON", grCamera)
        self.btnArucoDetect.setFont(self.defaultFontRegular)
        self.btnArucoDetect.setFixedSize(WIDTH_SIZE_DEFAULT, HEIGHT_SIZE_DEFAULT)

        layoutLeftButton.addWidget(self.btnOpenCloseCamera, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.btnRGBCamera, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.btnBinaryCamera, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.btnDepthCamera, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.btnCaptureAruco, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.btnCaptureObject, alignment=Qt.AlignTop)
        layoutLeftButton.addWidget(self.txtImageCount, alignment=Qt.AlignTop)

        layoutRightButton.addWidget(self.btnYoloDetect, alignment=Qt.AlignTop)
        layoutRightButton.addWidget(self.btnColorDetect, alignment=Qt.AlignTop)
        layoutRightButton.addWidget(self.btnArucoDetect, alignment=Qt.AlignTop)

        self.frameDisplayCamera = QLabel(grCamera)
        self.frameDisplayCamera.setFrameShape(QFrame.Box)
        self.frameDisplayCamera.setFixedSize(640, 480)

        layoutCamera.addLayout(layoutLeftButton)
        layoutCamera.addWidget(self.frameDisplayCamera)
        layoutCamera.addLayout(layoutRightButton)
        grCamera.setLayout(layoutCamera)
        self.rightMainLayout.addWidget(grCamera)


class SwitchButton(QWidget):
    signalSwitchToggled = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.status = False
        self.btnBackground = QPushButton(self)
        self.btnBackground.setFixedSize(60, 30)
        self.btnBackground.setStyleSheet(""" 
            QPushButton {
                background-color: lightgray;
                border-radius: 15px;
            }""")
        self.btnCircle = QPushButton(self.btnBackground)
        self.btnCircle.setFixedSize(26, 30)
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
#     window.showMinimized()
#     sys.exit(app.exec_())
