# Huỳnh Tuấn Kiệt - 2010364
import sys
import os
import numpy as np

# image_folder_path = 'D:\\A_Project\\PyCharm_Project\\Image\\Data'
# image_files = [f for f in os.listdir(image_folder_path) if f.lower().endswith('.jpg')]

# Để nhân 2 ma trận ta dùng toán tử
# (A@B) hay A.dot(B)
# (B@A) hay B.dot(A)
# Ngoài ra để nhân các phần tử của ma trận A với các phần tử tương ứng của ma trận B thì ta dùng toán tử
# A*B (Lưu ý: Toán tử này chỉ dùng được khi kích thước của ma trận A và ma trận B là giống nhau)

A = np.array([[3, 6, 7], [5, -3, 0]])
B = np.array([[1, 1], [2, 1], [3, -3]])
C = A.dot(B)  # Toán tử nhân ma trận (A nhân B) khác với (B nhân A)

D = np.array([[1, 3, 4], [-2, 6, 0], [-5, 7, 2]])
E = np.array([[2, 3, 4], [-1, -2, -3], [0, 4, -4]])
F = D @ E  # Tương tự như toán tử dot
G = D * E  # Toán tử nhân từng phần tử của ma trận này với các phần tử tương ứng của ma trận kia

print("C = \n", C)
print("F = \n", F)
print("G = \n", G)
# print(len(image_files))
