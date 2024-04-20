import sys
import cv2
import imutils
import argparse
import math
import numpy as np

from imutils import perspective
from imutils import contours

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication, QTextEdit, QFileDialog

from YoloDetection import *
from YoloSegmentation import *
from Guide_test import *


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)

        self.yoloDetect = YoloDetection()
        self.yoloSegment = YoloSegmentation()

        self.frame = None
        self.binary = None
        self.contour_box = None

        # self.tl_point = (100, 275)
        # self.br_point = (300, 420)

        width = 640
        height = 480
        self.black_frame = np.zeros((height, width), dtype=np.uint8)

        self.uic.btn_Open.clicked.connect(self.open_action)
        self.uic.btn_Detect.clicked.connect(self.detect_action)
        self.uic.btn_Close.clicked.connect(self.close_action)

    def open_action(self):
        opt = QFileDialog.Options()
        opt |= QFileDialog.DontUseNativeDialog
        file, _ = QFileDialog.getOpenFileName(self, "Open Image", "", "Images (*.png *.jpg *.bmp *.gif)",
                                              options=opt)
        if file:
            cap = cv2.imread(file)

            self.frame = cv2.cvtColor(cap, cv2.COLOR_BGR2RGB)

            pixmap = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QImage.Format_RGB888)
            self.uic.Image_frame_1.setPixmap(QPixmap.fromImage(pixmap))

    def detect_action(self):
        conv = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        img, last_id, _, _, top_left, bottom_right = self.yoloDetect.getObject(conv)
        self.frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        binary_float32 = self.yoloSegment.getSegment(conv)
        if binary_float32 is None:
            self.binary = self.black_frame
        else:
            binary_normalized = cv2.normalize(binary_float32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.binary = binary_normalized.astype(np.uint8)

        pixmap = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QImage.Format_RGB888)
        self.uic.Image_frame_1.setPixmap(QPixmap.fromImage(pixmap))

        pixmap = QImage(self.binary, self.binary.shape[1], self.binary.shape[0], QImage.Format_Indexed8)
        self.uic.Image_frame_2.setPixmap(QPixmap.fromImage(pixmap))

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
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)

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

    def close_action(self):
        self.uic.Image_frame_1.clear()
        self.uic.Image_frame_2.clear()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())
