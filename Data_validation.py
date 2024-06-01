import sys
import os
import random
import shutil

# Đường dẫn đến thư mục chứa ảnh ban đầu
source_folder = 'D:\\A_Project\\PyCharm_Project\\Object_Detection_2\\images\\train'
# source_folder = 'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Data_(2)'

# Đường dẫn đến thư mục mới
destination_folder = 'D:\\A_Project\\PyCharm_Project\\Object_Detection_2\\images\\val'
# destination_folder = 'D:\\A_Project\\PyCharm_Project\\Camera_Calibration\\Data\\Image\\Test'

# Lấy danh sách tất cả các tệp trong thư mục ban đầu
all_files = os.listdir(source_folder)

# Lấy ngẫu nhiên 28 tệp từ danh sách tệp
random_files = random.sample(all_files, 15)

# Tạo thư mục mới nếu chưa tồn tại
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

# Sao chép các tệp đã chọn vào thư mục mới
for file in random_files:
    source_file = os.path.join(source_folder, file)
    destination_file = os.path.join(destination_folder, file)
    shutil.copy(source_file, destination_file)

print("Finished")
