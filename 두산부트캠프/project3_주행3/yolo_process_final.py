import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import json
from cv_bridge import CvBridge

class ObjectDetector:
    def __init__(self):
        self.K = np.array([
            [1.38911918e+03, 0.00000000e+00, 5.64684807e+02],
            [0.00000000e+00, 1.43630583e+03, 3.23436648e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        self.dist_coeffs = np.array([
            5.94270763e-02, 1.86582413e+00, -1.97749264e-02,
            -2.86616788e-02, -2.00933000e+01
        ])
        self.model = YOLO('/home/jeonghan/int3_manipulator_best.pt')

    def undistort_point(self, x, y):
        point = np.array([[x, y]], dtype=np.float32)
        undistorted = cv2.undistortPoints(point, self.K, self.dist_coeffs, P=self.K)
        return undistorted[0][0]

    def image_to_world(self, x, y, z):
        undistorted_x, undistorted_y = self.undistort_point(x, y)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        x_norm = (undistorted_x - cx) / fx
        y_norm = (undistorted_y - cy) / fy
        X = x_norm * z
        Y = y_norm * z
        return X, Y, z

    def detect_and_convert(self, image):
        colors = {0: (0, 0, 255), 1: (255, 0, 0), 2: (255, 0, 255)}
        image_center_x = image.shape[1] // 2
        image_center_y = image.shape[0] // 2

        results = self.model(image)
        if not results or not results[0].boxes:
            return image, [], False

        object_id = 1
        contains_red_or_blue = False
        contains_purple = False
        status = None

        base_to_cam = np.array([0, 204.5, 174])
        z_value = 199
        camera_pos_adjustment = np.array([-9.0, 31.0, 0.0])
        tcp_to_cam = [54.5, 94]

        for result in results:
            if not hasattr(result, 'boxes'):
                continue
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                if cls_name == "red" or cls_name == "blue":
                    contains_red_or_blue = True
                elif cls_name == "purple":
                    contains_purple = True

        if contains_red_or_blue and not contains_purple:
            base_to_cam = np.array([100 + tcp_to_cam[0], 0.0, 100 + tcp_to_cam[1]])
            z_value = 205
            camera_pos_adjustment = np.array([33.0, 10.0, 0.0])
            status = "intial_pick"
        elif contains_purple and not contains_red_or_blue:
            base_to_cam = np.array([0, 150 + tcp_to_cam[0], 80 + tcp_to_cam[1]])
            z_value = 199
            camera_pos_adjustment = np.array([9.0, 27.0, 0.0])
            status = "final_pick"

        cam_center_world_x, cam_center_world_y, cam_center_world_z = self.image_to_world(image_center_x, image_center_y, z_value)

        output_data = []
        for result in results:
            if not hasattr(result, 'boxes'):
                continue
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                object_center_x = (x1 + x2) // 2
                object_center_y = (y1 + y2) // 2
                cam_object_world_x, cam_object_world_y, cam_object_world_z = self.image_to_world(object_center_x, object_center_y, z_value)
                cam_to_obj = [cam_object_world_x - cam_center_world_x, cam_object_world_y - cam_center_world_y, cam_object_world_z]

                if status == "intial_pick":
                    global_x = base_to_cam[0] - cam_to_obj[1] + camera_pos_adjustment[0]
                    global_y = base_to_cam[1] - cam_to_obj[0] + camera_pos_adjustment[1]
                    global_z = base_to_cam[2] - cam_to_obj[2] + camera_pos_adjustment[2]
                elif status == "final_pick":
                    global_x = base_to_cam[0] + cam_to_obj[0] + camera_pos_adjustment[0]
                    global_y = base_to_cam[1] - cam_to_obj[1] + camera_pos_adjustment[1]
                    global_z = base_to_cam[2] - cam_to_obj[2] + camera_pos_adjustment[2]
                else:
                    continue

                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                color = colors.get(cls_id, (0, 255, 0))
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                output_data.append({
                    "id": object_id,
                    "color": cls_name,
                    "position": (round(global_x, 1), round(global_y, 1), round(global_z, 1)-20)
                })
                object_id += 1

        return image, output_data, contains_purple


class BoxProcessor:
    def __init__(self, color_count, goal_id, error_publisher):
        self.color_count = color_count
        self.goal_id = goal_id
        self.error_publisher = error_publisher

    def validate_color_requirements(self, data):
        available_colors = {}
        for obj in data:
            color = obj["color"]
            available_colors[color] = available_colors.get(color, 0) + 1

        for color, required_count in self.color_count.items():
            if available_colors.get(color, 0) < required_count:
                error_msg = String()
                error_msg.data = f"Invalid number of boxes: {color}"
                self.error_publisher.publish(error_msg)
                raise ValueError(f"Insufficient boxes: {color}")

    def filter_boxes_by_color_and_count(self, data):
        result = []
        remaining_colors = self.color_count.copy()
        for obj in data:
            color = obj["color"]
            position = obj["position"]
            if color in remaining_colors and remaining_colors[color] > 0:
                result.append({
                    "box_id": obj["id"],
                    "color": color,
                    "position": position
                })
                remaining_colors[color] -= 1
            if all(count <= 0 for count in remaining_colors.values()):
                break
        return result

    def format_final_output(self, filtered_boxes):
        return {"goal_id": self.goal_id, "boxes": filtered_boxes}

    def process_boxes(self, data):
        self.validate_color_requirements(data)
        filtered_boxes = self.filter_boxes_by_color_and_count(data)
        return self.format_final_output(filtered_boxes)


class YOLOImageProcessor(Node):
    def __init__(self):
        super().__init__('yolo_image_processor')
        self.get_logger().info("YOLOImageProcessor node initialized.")
        self.subscription = self.create_subscription(CompressedImage, 'robot/image', self.image_callback, 10)
        self.bridge = CvBridge()
        self.yolo_image_publisher = self.create_publisher(CompressedImage, 'yolo/image', 10)
        self.task_subscription = self.create_subscription(String, 'robot_task_and_goal', self.task_callback, 10)
        self.box_pick_publisher = self.create_publisher(String, 'box_pick_data', 10)
        self.error_publisher = self.create_publisher(String, 'error_msg', 10)

        self.pick_box_data = []
        self.goal_data = None
        self.ready_to_process = False

        self.get_logger().info("YOLOImageProcessor node ready.")

    def image_callback(self, msg):
        try:
            self.get_logger().info(f"Image received.... process start")
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            detector = ObjectDetector()
            self.result_image, self.pick_box_data, self.contains_purple = detector.detect_and_convert(image)

            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(self.result_image)
            self.yolo_image_publisher.publish(compressed_msg)

            if self.contains_purple:
                self.goal_data = {"color_count": {"purple": 1}, "goal_id": 1}
                self.ready_to_process = True

            if self.ready_to_process and self.pick_box_data:
                self.publish_box_pick_data()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def task_callback(self, msg):
        try:
            self.goal_data = json.loads(msg.data)
            self.get_logger().info(f"Task message received: {self.goal_data}")
            self.ready_to_process = True

            if self.pick_box_data:
                self.publish_box_pick_data()

        except Exception as e:
            self.get_logger().error(f"Task message processing error: {e}")

    def publish_box_pick_data(self):
        try:
            goal_id = self.goal_data.get("goal_id")
            color_count = self.goal_data.get("color_count")
            processor = BoxProcessor(color_count, goal_id, self.error_publisher)
            pick_list = processor.process_boxes(self.pick_box_data)

            output_msg = String()
            final_output = {
                "goal_id": goal_id,
                "contains_purple": self.contains_purple,
                "boxes": pick_list["boxes"]
            }
            self.get_logger().info(f"box_pick_data: {json.dumps(final_output, indent=4)}")

            output_msg.data = json.dumps(final_output)
            self.box_pick_publisher.publish(output_msg)

            self.ready_to_process = False
            self.pick_box_data = []

        except Exception as e:
            self.get_logger().error(f"box_pick_data publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLOImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
