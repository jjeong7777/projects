# transformation(local->global) , #5에서 완성 

    # input : 1. A(실제크기 앎), 2. B(A가 포함된 이미지), 3. camera intrinsic 

    # PNPRANSAC

    # output : local to global transformation 

# #6 부터 시각화

    # 실선은 공간 상에 카메라 , 점선은 map

    



import cv2

import numpy as np

import matplotlib.pyplot as plt





# 1. 사진 크기 및 3D 공간 좌표 설정

# 쓰러진 사람: 23cm * 18cm

photo_width = 0.23 # 사진의 실제 폭 (단위: m) 

photo_height = 0.18  # 사진의 실제 높이 (단위: m)



# 사진의 3D 좌표 (4개의 코너 점, 왼쪽 위부터 시계 방향)

object_points = np.array([

    [0, 0, 0],                   # Top-left corner

    [photo_width, 0, 0],         # Top-right corner

    [photo_width, photo_height, 0],  # Bottom-right corner

    [0, photo_height, 0]         # Bottom-left corner

], dtype=np.float32)



# 2. 카메라 이미지 상의 좌표 (픽셀 단위, 임의로 설정)

image_points = np.array([

    [53,254],  # Top-left corner

    [241,243],  # Top-right corner

    [256,384],  # Bottom-right corner

    [71,47]   # Bottom-left corner

], dtype=np.float32)



# 3. 카메라 내각 행렬

camera_matrix = np.array([

    [1059, 0, 359],  # fx, 0, cx

    [0, 1057, 184],  # 0, fy, cy

    [0, 0, 1]       # 0, 0, 1

], dtype=np.float32)



# 왜곡 계수 (여기서는 왜곡 없음으로 가정)

dist_coeffs = np.zeros((4, 1), dtype=np.float32)



# 4. PnP 해법으로 회전 벡터와 평행 이동 벡터 계산

success, rvec, tvec, inliers  = cv2.solvePnPRansac(

    object_points, image_points, camera_matrix, dist_coeffs

)



if success:

    print("Rotation Vector (rvec):", rvec.ravel())

    print("Translation Vector (tvec):", tvec.ravel())

else:

    print("PnP 해결에 실패했습니다.")



# 5. 회전 행렬로 변환

rotation_matrix, _ = cv2.Rodrigues(rvec)

transformation_matrix_3_4 = np.hstack((rotation_matrix, tvec))

transformation_matrix = np.vstack((transformation_matrix_3_4,[0, 0, 0, 1]) )





#6. Define unit vectors along x, y, z axes

origin = np.array([0, 0, 0])

unit_vectors = {

    "x": np.array([1, 0, 0]),

    "y": np.array([0, 1, 0]),

    "z": np.array([0, 0, 1])

}



# Extract the translation component

translation = transformation_matrix[:3, 3]



# Transform unit vectors

transformed_vectors = {}

for key, vec in unit_vectors.items():

    vec_homogeneous = np.append(vec, 1)  # Convert to homogeneous coordinates

    transformed_vec = transformation_matrix @ vec_homogeneous

    # Normalize the vector to retain original unit length (ignoring translation)

    transformed_vec[:3] -= translation  # Remove translation effect

    transformed_vec[:3] /= np.linalg.norm(transformed_vec[:3])  # Normalize

    transformed_vectors[key] = transformed_vec[:3]  # Convert back to Cartesian



# Plot original and transformed vectors including translation

fig = plt.figure(figsize=(10, 8))

ax = fig.add_subplot(111, projection='3d')



# Plot original vectors

colors = {"x": "r", "y": "g", "z": "b"}

for key, vec in unit_vectors.items():

    ax.quiver(*origin, *vec, color=colors[key], label=f'Original {key}-axis')



# Plot transformed vectors

for key, vec in transformed_vectors.items():

    ax.quiver(*translation, *vec, color=colors[key], linestyle="dashed", label=f'Transformed {key}-axis')



# Set labels and legend

ax.set_xlim([-2, 2])

ax.set_ylim([-2, 2])

ax.set_zlim([-2, 2])

ax.set_xlabel('X-axis')

ax.set_ylabel('Y-axis')

ax.set_zlabel('Z-axis')

ax.legend()

ax.set_title('Original and Transformed Unit Vectors (with Translation)')



plt.show()