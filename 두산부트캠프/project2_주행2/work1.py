# 값이 없으면 추가,i final_transforms가 id,변환 정보 담고 있음.

# self.visualize_pgm_with_transforms(self.final_transforms)

# final_transform1 - 'green' # man

# final_transform2 - 'red' # extinguisher

#

# pgm_file_path = "/home/dj/Downloads/my_map.pgm" 수정 필요 

# 



import cv2

import numpy as np

from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import PointStamped, TransformStamped

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge

import time

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from tf_transformations import quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix

import matplotlib.pyplot as plt

from PIL import Image



def match_keypoints(desc1, desc2, kp1, kp2, distance_threshold=110):

    """

    특징점 매칭 및 신뢰도 높은 매칭만 선택

    """

    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    matches = bf.match(desc1, desc2)

    matches = sorted(matches, key=lambda x: x.distance)

    good_matches = [m for m in matches if m.distance < distance_threshold]

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    return src_pts, dst_pts, good_matches



camera_matrix = np.array([

    [200.5153350830078, 0.0, 125.41188049316406],

    [0.0, 200.5153350830078, 126.19374084472656],

    [0.0, 0.0, 1.0]

], dtype=np.float32)



dist_coeffs = np.array([

    7.169227123260498,

    -91.49100494384766,

    0.0012667347909882665,

    0.0013030755799263716,

    300.4808044433594,

    7.03811502456665,

    -90.58170318603516,

    297.64654541015625

], dtype=np.float32)



target_path1 = "/home/namsang/project/man_orig.png"

target_path2 = "/home/namsang/project/ext_orig.png"

gray_target1 = cv2.imread(target_path1, cv2.IMREAD_GRAYSCALE)

gray_target2 = cv2.imread(target_path2, cv2.IMREAD_GRAYSCALE)

# Specify image dimensions for each target

image_dimensions = {

    1: {"width": 869, "height": 680, "photo_width": 0.23, "photo_height": 0.18}, # man 

    2: {"width": 680, "height": 480, "photo_width": 0.2, "photo_height": 0.15} # ext 

}



class ImageProcessor(Node):

    def __init__(self):

        super().__init__('image_processor')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(

            CompressedImage,

            '/oakd/rgb/preview/image_raw/compressed',

            self.image_callback,

            10

        )

        self.point_pub = self.create_publisher(PointStamped, '/transformed_points', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = time.time()

        # Initialize TF2

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = "map"

        self.source_frame = "oakd_rgb_camera_optical_frame"

        self.final_transforms = {}

        self.processed_targets = {1: False, 2: False}

        

    def get_transform_matrix(self):

        """

        Retrieve the transform from the camera frame to the map frame.

        """

        try:

            transform = self.tf_buffer.lookup_transform(

                self.target_frame,

                self.source_frame,

                rclpy.time.Time()

            )

            translation = transform.transform.translation

            rotation = transform.transform.rotation

            translation_matrix_np = translation_matrix(

                (translation.x, translation.y, translation.z)

            )

            rotation_matrix_np = quaternion_matrix(

                (rotation.x, rotation.y, rotation.z, rotation.w)

            )

            transform_matrix = concatenate_matrices(translation_matrix_np, rotation_matrix_np)



            self.get_logger().info(f"Transform Matrix (Camera to Map):\n{transform_matrix}")

            return transform_matrix

        except Exception as e:

            self.get_logger().error(f"Could not get transform: {e}")

            return None

        

    def broadcast_transform(self, transform_matrix, frame_id, child_frame_id):

        """

        Publish the final transform as a TF.

        """

        translation = translation_from_matrix(transform_matrix)

        rotation = quaternion_from_matrix(transform_matrix)

        

        # 변환 정보 로깅

        self.get_logger().info(f"Broadcasted Transform ({child_frame_id}):")

        self.get_logger().info(f"Translation: x={translation[0]}, y={translation[1]}, z={translation[2]}")

        self.get_logger().info(f"Rotation: x={rotation[0]}, y={rotation[1]}, z={rotation[2]}, w={rotation[3]}")

        

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()

        transform.header.frame_id = frame_id

        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = translation[0]

        transform.transform.translation.y = translation[1]

        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]

        transform.transform.rotation.y = rotation[1]

        transform.transform.rotation.z = rotation[2]

        transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(f"Broadcasted Transform: {frame_id} -> {child_frame_id}")

    

    def visualize_pgm_with_transforms(final_transforms):

        """

        PGM 파일과 final_transforms를 사용하여 변환 위치를 시각화하는 함수.



        Parameters:

            pgm_file_path (str): PGM 파일 경로

            final_transforms (dict): 변환 정보를 담은 딕셔너리 (키는 문자열, 값은 4x4 행렬)

        

        Output:

            화면에 변환 좌표를 표시한 이미지 출력

        """

        # 1. PGM 파일 불러오기

        def load_pgm(file_path):

            with open(file_path, 'rb') as f:

                pgm_image = Image.open(f)

                return np.array(pgm_image)

        pgm_file_path = "/home/dj/Downloads/my_map.pgm"

        # Load PGM image

        image = load_pgm(pgm_file_path)

        

        # 2. PGM 이미지를 출력

        plt.figure(figsize=(10, 10))

        plt.imshow(image, cmap='gray')

        

        # 3. 각 transform의 translation 위치를 표시

        for key, transform in final_transforms.items():

            # Translation 값 (x, y)

            x, y = transform[0, 3], transform[1, 3]

            

            if key == "final_transform1":

                plt.scatter(x, y, c='green', label='final_transform1', s=100)

                plt.gca().add_patch(plt.Rectangle((x-2, y-2), 4, 4, edgecolor='green', facecolor='none', linewidth=2))

            elif key == "final_transform2":

                plt.scatter(x, y, c='red', label='final_transform2', s=100)

                plt.gca().add_patch(plt.Circle((x, y), 3, color='red', fill=False, linewidth=2))

            else:

                plt.scatter(x, y, c='blue', label=key, s=100)  # 다른 key는 파란색 점으로 표시

        

        # 4. 레이블 추가 및 표시

        plt.legend()

        plt.title("Final Transforms Visualization")

        plt.show()



            

    def process_target(self, gray_target, cam, target_id):

        sift = cv2.SIFT_create()

        kp_target, desc_target = sift.detectAndCompute(gray_target, None)

        kp_cam, desc_cam = sift.detectAndCompute(cam, None)

        if desc_target is None or desc_cam is None or len(desc_cam) < 1:

            self.get_logger().warn(f"Feature extraction failed for target {target_id}.")

            return

        src_pts, dst_pts, matches = match_keypoints(desc_target, desc_cam, kp_target, kp_cam, distance_threshold=150)

        if len(matches) >= 35:

            target_params = image_dimensions[target_id]

            target_image_width = target_params["width"]

            target_image_height = target_params["height"]

            target_photo_width = target_params["photo_width"]

            target_photo_height = target_params["photo_height"]

            src_pts_xy = src_pts.reshape(-1, 2)

            x_coords = src_pts_xy[:, 0]

            y_coords = src_pts_xy[:, 1]

            scaled_x_coords = x_coords / target_image_width * target_photo_width

            scaled_y_coords = y_coords / target_image_height * target_photo_height

            scaled_src_pts = np.stack((scaled_x_coords, scaled_y_coords), axis=1)

            real_src_pts = np.hstack((scaled_src_pts, np.zeros((scaled_src_pts.shape[0], 1), dtype=np.float32)))

            

            success, rvec, tvec, inliers = cv2.solvePnPRansac(real_src_pts, dst_pts, camera_matrix, dist_coeffs)

            if success:

                self.get_logger().info(f"PnP Result - rvec: {rvec}, tvec: {tvec}")



                rotation_matrix, _ = cv2.Rodrigues(rvec)

                transformation_matrix_3_4 = np.hstack((rotation_matrix, tvec))

                cam_to_target_transformation_matrix = np.vstack((transformation_matrix_3_4, [0, 0, 0, 1]))

                tf_cam_to_target = np.linalg.inv(cam_to_target_transformation_matrix)

                

                # 로그에 타임스탬프 추가

                timestamp_tf_cam_to_target = self.get_clock().now().to_msg()  # ROS2 타임스탬프 얻기

                self.get_logger().info(f"Timestamp after computing tf_cam_to_target: {timestamp_tf_cam_to_target.sec}.{timestamp_tf_cam_to_target.nanosec}")

                self.get_logger().info(f"Transform Matrix (Camera to Target):\n{tf_cam_to_target}")



                map_transform_matrix = self.get_transform_matrix()

                

                # 로그에 타임스탬프 추가

                timestamp_map_transform_matrix = self.get_clock().now().to_msg()  # ROS2 타임스탬프 얻기

                self.get_logger().info(f"Timestamp after computing map_transform_matrix: {timestamp_map_transform_matrix.sec}.{timestamp_map_transform_matrix.nanosec}")

                

                if map_transform_matrix is not None:

                    final_transform = map_transform_matrix @ cam_to_target_transformation_matrix

                    self.get_logger().info(f"Final Transform Matrix (Map to Target):\n{final_transform}")



                    # ***************값이 없으면 추가,i final_transforms가 id,변환 정보 담고 있음.

                    if target_id in self.final_transforms:

                        print(f"Target ID {target_id} already exists. Not updating the value.")

                    else:

                        # 값이 없으면 추가

                        self.final_transforms[f"final_transform{target_id}"] = final_transform

                        print(f"Added final_transform for target ID {target_id}.")

                    

                    # 함수 실행

                    self.visualize_pgm_with_transforms(self.final_transforms)

                        

                    self.broadcast_transform(final_transform, "map", f"final_transform{target_id}")

                    

                    # Camera to Map Transform Broadcast

                    self.broadcast_transform(map_transform_matrix, "map", "transform_camera")

                    

                    self.processed_targets[target_id] = True

    

                    

    def image_callback(self, msg):

        current_time = time.time()

        if current_time - self.last_time < 1 / 30.0:  # Limit to 30 FPS

            return

        self.last_time = current_time

        try:

            cam = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cam is None or cam.size == 0:

                self.get_logger().warn("Empty image received.")

                return

            gray_cam = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)

            if gray_cam is None or gray_cam.size == 0:

                self.get_logger().warn("Grayscale conversion resulted in an empty image.")

                return

            self.process_target(gray_target1, gray_cam, 1)

            self.process_target(gray_target2, gray_cam, 2)

            if all(self.processed_targets.values()):

                for name, matrix in self.final_transforms.items():

                    self.get_logger().info(f"{name}:")

                    self.get_logger().info(f"\n{matrix}")

                self.processed_targets = {1: False, 2: False}  # Reset for the next cycle

            if cv2.waitKey(1) & 0xFF == 27:

                self.get_logger().info("User exit.")

                rclpy.shutdown()

        except Exception as e:

            self.get_logger().error(f"Error processing image: {e}")

            

def main(args=None):

    rclpy.init(args=args)

    image_processor = ImageProcessor()

    rclpy.spin(image_processor)

    cv2.destroyAllWindows()

    rclpy.shutdown()



if __name__ == '__main__':

    main()