from __future__ import print_function  # Python 2/3 compatibility
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys  # Import system library

# Set the desired ArUco dictionary
desired_aruco_dictionary = "DICT_5X5_100"

# The different ArUco dictionaries built into the OpenCV library
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Example calibration matrix and distortion coefficients
#matrix_coefficients = np.array([[933.15867, 0, 657.59], [0, 933.1586, 400.36993], [0, 0, 1]])
#distortion_coefficients = np.array([-0.43948, 0.18514, 0, 0, 0])

matrix_coefficients = np.array([
    [945.30655017, 0., 397.09171751],
    [0., 943.30966751, 290.97935771],
    [0., 0., 1.]
])

distortion_coefficients = np.array([
    [0.00909008, 0.53074901, -0.00409163, 0.00573666, -1.68100495]
])

'''
matrix_coefficients = np.array([[663.53906823,   0.,         620.95123764], 
                [  0.,         664.13958877, 353.79669005],
                [  0.,           0.,           1.        ]])

distortion_coefficients = np.array([[-0.00746391,  0.06038115, -0.0001621,   0.00045486, -0.04346805]])
'''
'''
matrix_coefficients = np.array([
    [800.61537097, 0., 404.73695332],
    [0., 798.40717274, 313.93967731],
    [0., 0., 1.]
])

distortion_coefficients = np.array([[ 5.18712249e-02 ,-7.46193055e-01 ,-1.29482120e-02 , 3.01939407e-04 ,7.38770249e+00]])
'''

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # 이미지 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/aruco', 10) # /image_raw/compressed
        #self.publisher_ = self.create_publisher(CompressedImage, '/image_raw/compressed', 10) # /image_raw/compressed
        
        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        self.timer = self.create_timer(0.1, self.publish_image)

        #초깃값 설정 
        #self.intrinsic_camera = np.array(((438.783367, 0.000000, 305.593336), (0.000000, 437.302876, 243.738352), (0, 0, 1)))
        #self.distortion = np.array((-0.361976, 0.110510, 0.001014, 0.000505))
        
        #self.intrinsic_camera = np.array(((1059, 0, 359), (0, 1057, 184), (0, 0, 1)))
        #self.distortion = np.array((-0.43948, 0.18514, 0, 0))
        
        # OpenCV 비디오 캡처 객체 생성 (카메라 2번 장치 사용)
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 해상도값 출력
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        
    def publish_image(self):
            # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(desired_aruco_dictionary))
            sys.exit(0)

        # Load the ArUco dictionary
        print("[INFO] Detecting '{}' markers...".format(desired_aruco_dictionary))
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters()
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()

            if not ret:
                print("[ERROR] Could not read frame. Exiting...")
                break

            # Detect ArUco markers in the video frame
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, this_aruco_dictionary, parameters=this_aruco_parameters)

            # Check that at least one ArUco marker was detected
            if len(corners) > 0:
                # Flatten the ArUco IDs list
                ids = ids.flatten()

                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Loop over the detected ArUco corners
                for (marker_corner, marker_id) in zip(corners, ids):
                    
                    # Estimate pose of marker
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                        marker_corner, 0.05, matrix_coefficients, distortion_coefficients)

                    if marker_id == 76 or marker_id == 74: 
                        print(f" marker_id : {marker_id}, rvec : {rvec[0]}")
                    
                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec[0], tvec[0], 0.05)
                    
            #--------------------압축--------------------------------------
            # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing compressed image...')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    
    # ROS 2 노드 실행
    rclpy.spin(image_publisher)

    # 종료 시 리소스 해제
    image_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
