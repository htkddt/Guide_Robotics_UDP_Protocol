# Huỳnh Tuấn Kiệt - 2010364
import sys
import os
import cv2
# Test source // Test source

tl_point = (150, 275)
br_point = (470, 420)

image_folder_path = 'D:\\A_Project\\PyCharm_Project\\Object_Detection_2\\images\\val'
image_files = [f for f in os.listdir(image_folder_path) if f.lower().endswith('.jpg')]

result_folder_path = 'D:\\A_Project\\PyCharm_Project\\Object_Detection_3\\images\\val'

for index in range(1, len(image_files) + 1):
    image = f'D:\\A_Project\\PyCharm_Project\\Object_Detection_2\\images\\val\\image_({index}).jpg'

    # print(str(index))
    # print(str(image))

    img = cv2.imread(image)

    cropped_frame = img[tl_point[1]:br_point[1], tl_point[0]:br_point[0]]

    cropped_file_name = f'image_({index}).jpg'
    cv2.imwrite(os.path.join(result_folder_path, cropped_file_name), cropped_frame)
