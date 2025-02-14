import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QPushButton, QTextEdit, QSlider, QLabel, QLineEdit
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGroupBox
from PyQt5.QtWidgets import QMessageBox, QSizePolicy
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QUrl, QSize, QRect, QCoreApplication


class MainWindowUI(object):
    def initUI(self, mainWindow):
        mainWindow.setObjectName("MainWindow")
        mainWindow.resize(1520, 900)
        mainWindow.setWindowTitle("Robot Application")

        self.initConnectionGroup(mainWindow)
        self.initBasicControlGroup(mainWindow)
        self.initIncrementalControlGroup(mainWindow)
        self.initGetPositionGroup(mainWindow)
        self.initJobControlGroup(mainWindow)
        self.initCameraGroup(mainWindow)

    def initConnectionGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grConnection = QGroupBox(mw)
        grConnection.setTitle("CONNECTION")
        grConnection.setFont(fontBold)
        grConnection.setGeometry(30, 10, 561, 251)

        # Group Robot inside Group Connection
        grRobot = QGroupBox(grConnection)
        grRobot.setTitle("ROBOT")
        grRobot.setFont(fontBold)
        grRobot.setGeometry(20, 20, 251, 221)

        self.lbIPRobot = QLabel(grRobot)
        self.lbIPRobot.setFont(fontRegular)
        self.lbIPRobot.setAlignment(Qt.AlignLeft)
        self.lbIPRobot.setGeometry(10, 30, 55, 16)

        self.txtIPRobot = QLineEdit(grRobot)
        self.txtIPRobot.setFont(fontBold)
        self.txtIPRobot.setAlignment(Qt.AlignLeft)
        self.txtIPRobot.setGeometry(80, 30, 161, 22)

        self.lbPortRobot = QLabel(grRobot)
        self.lbPortRobot.setFont(fontRegular)
        self.lbPortRobot.setAlignment(Qt.AlignLeft)
        self.lbPortRobot.setGeometry(10, 60, 55, 16)

        self.txtPortRobot = QLineEdit(grRobot)
        self.txtPortRobot.setFont(fontBold)
        self.txtPortRobot.setAlignment(Qt.AlignLeft)
        self.txtPortRobot.setGeometry(80, 60, 161, 22)

        self.btnRobotConDis = QPushButton(grRobot)
        self.btnRobotConDis.setGeometry(10, 90, 231, 41)

        self.lbDeviceRobotStatus = QLabel(grRobot)
        self.lbDeviceRobotStatus.setFont(fontRegular)
        self.lbDeviceRobotStatus.setAlignment(Qt.AlignLeft)
        self.lbDeviceRobotStatus.setGeometry(10, 150, 131, 20)

        self.btnDeviceRobotBackground = QPushButton(grRobot)
        self.btnDeviceRobotBackground.setGeometry(150, 142, 60, 30)
        self.btnDeviceRobotBackground.setStyleSheet(""" 
            QPushButton {
                background-color: lightgray;
                border-radius: 15px;
            }""")
        self.btnDeviceRobotCircle = QPushButton(self.btnDeviceRobotBackground)
        self.btnDeviceRobotCircle.setGeometry(0, 0, 26, 30)
        self.btnDeviceRobotCircle.setStyleSheet("""
            QPushButton {
                background-color: white;
                border-radius: 13px;
            }""")

        self.lbRobotConnectStatus = QLabel(grRobot)
        self.lbRobotConnectStatus.setFont(fontRegular)
        self.lbRobotConnectStatus.setAlignment(Qt.AlignLeft)
        self.lbRobotConnectStatus.setGeometry(10, 190, 131, 20)

        self.txtRobotConnectStatus = QLabel(grRobot)
        self.txtRobotConnectStatus.setFont(fontBold)
        self.txtRobotConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtRobotConnectStatus.setGeometry(150, 190, 81, 20)

        # Group Gripper inside Group Connection
        grGripper = QGroupBox(grConnection)
        grGripper.setTitle("GRIPPER")
        grGripper.setFont(fontBold)
        grGripper.setGeometry(290, 20, 251, 221)

        self.lbPortGripper = QLabel(grGripper)
        self.lbPortGripper.setFont(fontRegular)
        self.lbPortGripper.setAlignment(Qt.AlignLeft)
        self.lbPortGripper.setGeometry(10, 30, 55, 16)

        self.txtPortGripper = QLineEdit(grGripper)
        self.txtPortGripper.setFont(fontBold)
        self.txtPortGripper.setAlignment(Qt.AlignLeft)
        self.txtPortGripper.setGeometry(80, 30, 161, 22)

        self.lbBaudGripper = QLabel(grGripper)
        self.lbBaudGripper.setFont(fontRegular)
        self.lbBaudGripper.setAlignment(Qt.AlignLeft)
        self.lbBaudGripper.setGeometry(10, 60, 55, 16)

        self.txtBaudGripper = QLineEdit(grGripper)
        self.txtBaudGripper.setFont(fontBold)
        self.txtBaudGripper.setAlignment(Qt.AlignLeft)
        self.txtBaudGripper.setGeometry(80, 60, 161, 22)

        self.btnGripperConDis = QPushButton(grGripper)
        self.btnGripperConDis.setGeometry(10, 90, 231, 41)

        self.lbDeviceIOStatus = QLabel(grGripper)
        self.lbDeviceIOStatus.setFont(fontRegular)
        self.lbDeviceIOStatus.setAlignment(Qt.AlignLeft)
        self.lbDeviceIOStatus.setGeometry(10, 150, 131, 20)

        self.btnDeviceIOBackground = QPushButton(grGripper)
        self.btnDeviceIOBackground.setGeometry(150, 142, 60, 30)
        self.btnDeviceIOBackground.setStyleSheet(""" 
                    QPushButton {
                        background-color: lightgray;
                        border-radius: 15px;
                    }""")
        self.btnDeviceIOCircle = QPushButton(self.btnDeviceIOBackground)
        self.btnDeviceIOCircle.setGeometry(0, 0, 26, 30)
        self.btnDeviceIOCircle.setStyleSheet("""
                    QPushButton {
                        background-color: white;
                        border-radius: 13px;
                    }""")

        self.lbIOConnectStatus = QLabel(grGripper)
        self.lbIOConnectStatus.setFont(fontRegular)
        self.lbIOConnectStatus.setAlignment(Qt.AlignLeft)
        self.lbIOConnectStatus.setGeometry(10, 190, 131, 20)

        self.txtIOConnectStatus = QLabel(grGripper)
        self.txtIOConnectStatus.setFont(fontBold)
        self.txtIOConnectStatus.setAlignment(Qt.AlignLeft)
        self.txtIOConnectStatus.setGeometry(150, 190, 81, 20)

    def initBasicControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grBasicControl = QGroupBox(mw)
        grBasicControl.setTitle("BASIC CONTROL")
        grBasicControl.setFont(fontBold)
        grBasicControl.setGeometry(30, 280, 561, 131)

    def initIncrementalControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grIncremental = QGroupBox(mw)
        grIncremental.setTitle("INCREMENTAL POSITION")
        grIncremental.setFont(fontBold)
        grIncremental.setGeometry(30, 430, 561, 421)

    def initGetPositionGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grGetPosition = QGroupBox(mw)
        grGetPosition.setTitle("GET POSITION")
        grGetPosition.setFont(fontBold)
        grGetPosition.setGeometry(610, 10, 481, 301)

    def initJobControlGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grJobControl = QGroupBox(mw)
        grJobControl.setTitle("JOB CONTROL")
        grJobControl.setFont(fontBold)
        grJobControl.setGeometry(1110, 10, 371, 301)

    def initCameraGroup(self, mw):
        fontBold = QFont("Time New Roman", 8)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 8)
        fontRegular.setBold(False)

        grCamera = QGroupBox(mw)
        grCamera.setTitle("CAMERA")
        grCamera.setFont(fontBold)
        grCamera.setGeometry(610, 320, 871, 531)

    def initText(self):
        fontBold = QFont("Time New Roman", 9)
        fontBold.setBold(True)
        fontRegular = QFont("Time New Roman", 9)
        fontRegular.setBold(False)

        # Group Robot
        self.lbIPRobot.setText("IP Address")
        self.lbPortRobot.setText("Port")
        self.txtIPRobot.setText("192.168.1.13")
        self.txtPortRobot.setText("10040")

        self.btnRobotConDis.setFont(fontBold)
        self.btnRobotConDis.setText("Connect")

        self.lbDeviceRobotStatus.setText("Device Robot Status:")
        self.lbRobotConnectStatus.setText("Robot Connection Status:")
        self.txtRobotConnectStatus.setText("Disconnected")

        # Group Gripper
        self.lbPortGripper.setText("Port")
        self.lbBaudGripper.setText("Baudrate")
        self.txtPortGripper.setText("COM4")
        self.txtBaudGripper.setText("115200")

        self.btnGripperConDis.setFont(fontBold)
        self.btnGripperConDis.setText("Connect")

        self.lbDeviceIOStatus.setText("Device I/O Status:")
        self.lbIOConnectStatus.setText("I/O Connection Status:")
        self.txtIOConnectStatus.setText("Disconnected")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QMainWindow()
    uic = MainWindowUI()
    uic.initUI(window)
    uic.initText()
    window.show()
    sys.exit(app.exec_())