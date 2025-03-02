import sys
import cv2
import imutils
import argparse
import math
import numpy as np

from imutils import perspective
from imutils import contours

from ultralytics import YOLO


class YoloDetection:
    def __init__(self):
        # Load model
        # train (epoch=20 và batch=2) Object_Detection_1
        # self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/detect/train/weights/best.pt")
        # train2 (epoch=20 và batch=2) Object_Detection_1 (last.pt model train)
        self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/detect/train2/weights/best.pt")
        # train3 (epoch=20 và batch=2) Object_Detection_2 (last.pt model train)
        # self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/detect/train3/weights/best.pt")
        # train4 (epoch=20 và batch=2) Object_Detection_2 (best.pt model train)
        # self.model = YOLO("D:/A_Project/PyCharm_Project/ApplicationRobotics/runs/detect/train4/weights/best.pt")

        # Load class
        self.license_class = [0, 1, 2, 3, 4]

        # Config
        self.results_list = {}
        self.frame_nr = -1

        print("Loading Yolo Detection Model")

    def getObject(self, frame, minPos, maxPos):
        min_X = minPos[0]
        max_X = maxPos[0]
        min_Y = minPos[1]
        max_Y = maxPos[1]

        point_center = None
        top_left = None
        bottom_right = None
        last_id = None
        name_last_id = None
        id_list = []

        if frame is not None:
            self.frame_nr += 1
            self.results_list[self.frame_nr] = {}

            # Detect
            license_results = self.model(frame)[0]
            license_plates = []

            for result in license_results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result

                if int(class_id) in self.license_class:
                    license_plates.append([x1, y1, x2, y2, score, class_id])

            # Draw bounding boxes on the image
            for result in license_plates:
                x1, y1, x2, y2, score, class_id = result

                point_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                if ((point_center[0] > min_X) & (point_center[0] < max_X) &
                    (point_center[1] > min_Y) & (point_center[1] < max_Y)):

                    top_left = (int(x1), int(y1))
                    bottom_right = (int(x2), int(y2))

                    cv2.circle(frame, point_center, 3, (0, 0, 0), -1)
                    cv2.circle(frame, top_left, 3, (0, 0, 0), 2)
                    cv2.circle(frame, bottom_right, 3, (0, 0, 0), 2)

                    if int(class_id) == 0:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)

                        cv2.putText(frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

                        cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

                    elif int(class_id) == 1:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 185, 75), 2)

                        cv2.putText(frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)

                        cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)

                    elif int(class_id) == 2:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (2, 88, 230), 2)

                        cv2.putText(frame, str(round(score, 2)), (int(x1 + 30), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)

                        cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)

                    elif int(class_id) == 3:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (167, 72, 217), 2)

                        cv2.putText(frame, str(round(score, 2)), (int(x1 + 45), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)

                        cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)

                    elif int(class_id) == 4:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 39, 205), 2)

                        cv2.putText(frame, str(round(score, 2)), (int(x1 + 55), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)

                        cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)

                    last_id = int(class_id)
                    name_last_id = license_results.names[int(class_id)].upper()
                    id_list.append(int(class_id))

                # top_left = (int(x1), int(y1))
                # bottom_right = (int(x2), int(y2))
                #
                # cv2.circle(frame, point_center, 3, (0, 0, 0), -1)
                # cv2.circle(frame, top_left, 3, (0, 0, 0), 2)
                # cv2.circle(frame, bottom_right, 3, (0, 0, 0), 2)
                #
                # if int(class_id) == 0:
                #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
                #
                #     cv2.putText(frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
                #
                #     cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
                #
                # elif int(class_id) == 1:
                #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 185, 75), 2)
                #
                #     cv2.putText(frame, str(round(score, 2)), (int(x1 + 35), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)
                #
                #     cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 185, 75), 1, cv2.LINE_AA)
                #
                # elif int(class_id) == 2:
                #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (2, 88, 230), 2)
                #
                #     cv2.putText(frame, str(round(score, 2)), (int(x1 + 30), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)
                #
                #     cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (2, 88, 230), 1, cv2.LINE_AA)
                #
                # elif int(class_id) == 3:
                #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (167, 72, 217), 2)
                #
                #     cv2.putText(frame, str(round(score, 2)), (int(x1 + 45), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)
                #
                #     cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 72, 217), 1, cv2.LINE_AA)
                #
                # elif int(class_id) == 4:
                #     cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (1, 39, 205), 2)
                #
                #     cv2.putText(frame, str(round(score, 2)), (int(x1 + 55), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)
                #
                #     cv2.putText(frame, license_results.names[int(class_id)].upper(), (int(x1), int(y1 - 5)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, (1, 39, 205), 1, cv2.LINE_AA)
                #
                # last_id = int(class_id)
                # name_last_id = license_results.names[int(class_id)].upper()
                # id_list.append(int(class_id))

        return frame, last_id, name_last_id, point_center, top_left, bottom_right, id_list
