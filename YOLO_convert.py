import sys
import cv2
import os

input_dir = 'D:\\A_Project\\PyCharm_Project\\Segmentation_Object\\tmp\\masks'
input_files = [f for f in os.listdir(input_dir) if f.lower().endswith('.png')]

output_dir = 'D:\\A_Project\\PyCharm_Project\\Segmentation_Object\\tmp\\convert'

for index in range(1, len(input_files) + 1):
    image = f'D:\\A_Project\\PyCharm_Project\\Segmentation_Object\\tmp\\masks\\image_({index}).png'
    mask = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)

    H, W = mask.shape
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    polygons = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 200:
            polygon = []
            for point in cnt:
                x, y = point[0]
                polygon.append(x / W)
                polygon.append(y / H)
            polygons.append(polygon)

    name = f'image_({index}).txt'
    path = os.path.join(output_dir, name)

    # print the polygons
    with open(path, 'w') as file:
        for polygon in polygons:
            for pol_, pol in enumerate(polygon):
                if pol_ == len(polygon) - 1:
                    file.write('{}\n'.format(pol))
                elif pol_ == 0:
                    file.write('0 {} '.format(pol))
                else:
                    file.write('{} '.format(pol))
