# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Guide_test.ui'
#
# Created by: PyQt5 UI code generator 5.15.10
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1311, 868)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.Image_frame_1 = QtWidgets.QLabel(self.centralwidget)
        self.Image_frame_1.setGeometry(QtCore.QRect(10, 60, 640, 480))
        self.Image_frame_1.setFrameShape(QtWidgets.QFrame.Box)
        self.Image_frame_1.setText("")
        self.Image_frame_1.setAlignment(QtCore.Qt.AlignCenter)
        self.Image_frame_1.setObjectName("Image_frame_1")
        self.btn_Open = QtWidgets.QPushButton(self.centralwidget)
        self.btn_Open.setGeometry(QtCore.QRect(800, 120, 111, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.btn_Open.setFont(font)
        self.btn_Open.setObjectName("btn_Open")
        self.btn_Close = QtWidgets.QPushButton(self.centralwidget)
        self.btn_Close.setGeometry(QtCore.QRect(1040, 120, 111, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.btn_Close.setFont(font)
        self.btn_Close.setObjectName("btn_Close")
        self.btn_Detect = QtWidgets.QPushButton(self.centralwidget)
        self.btn_Detect.setGeometry(QtCore.QRect(920, 120, 111, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.btn_Detect.setFont(font)
        self.btn_Detect.setObjectName("btn_Detect")
        self.Image_frame_2 = QtWidgets.QLabel(self.centralwidget)
        self.Image_frame_2.setGeometry(QtCore.QRect(660, 320, 640, 480))
        self.Image_frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.Image_frame_2.setText("")
        self.Image_frame_2.setAlignment(QtCore.Qt.AlignCenter)
        self.Image_frame_2.setObjectName("Image_frame_2")
        self.toolButton = QtWidgets.QToolButton(self.centralwidget)
        self.toolButton.setGeometry(QtCore.QRect(10, 0, 241, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.toolButton.setFont(font)
        self.toolButton.setObjectName("toolButton")
        self.toolButton_2 = QtWidgets.QToolButton(self.centralwidget)
        self.toolButton_2.setGeometry(QtCore.QRect(260, 0, 241, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.toolButton_2.setFont(font)
        self.toolButton_2.setObjectName("toolButton_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1311, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btn_Open.setText(_translate("MainWindow", "Open"))
        self.btn_Close.setText(_translate("MainWindow", "Close"))
        self.btn_Detect.setText(_translate("MainWindow", "Detect"))
        self.toolButton.setText(_translate("MainWindow", "Image Detection"))
        self.toolButton_2.setText(_translate("MainWindow", "Video Tracking"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
