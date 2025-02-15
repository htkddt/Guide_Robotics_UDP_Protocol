import sys
import cv2
import imutils
import argparse
import math

from imutils import perspective
from imutils import contours

from ultralytics import YOLO


class YoloSegmentation:
    def __init__(self):
        # Load model
        # train (epoch=20 và batch=2) Segment_Object
        # self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/segment/train/weights/best.pt")
        # train2 (epoch=20 và batch=2) Segment_Object (last.pt model train)
        self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/segment/train2/weights/best.pt")

        print("Loading Yolo Segmentation Model")

    def getSegment(self, frame):
        binary_frame = None

        if frame is not None:
            # Segment
            license_results = self.model(frame)[0]

            for result in license_results:
                for j, mask in enumerate(result.masks.data):
                    mask = mask.cpu()
                    mask = mask.numpy() * 255
                    binary_frame = mask

                    return binary_frame
