import sys
import json
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, Point, Quaternion
#from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from example_interfaces.srv import Trigger

class RobotStatusGUI(QWidget):
    update_status_signal = pyqtSignal(str)
    enable_next_goal_signal = pyqtSignal()
    emergency_stop_signal = pyqtSignal()
    restart_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Status")
        self.setGeometry(100, 100, 400, 400)

        # Layout and Widgets
        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignCenter)

        self.status_label = QLabel("Waiting for command...", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)

        self.background_label = QLabel(self)
        self.background_label.setScaledContents(True)
        self.layout.addWidget(self.background_label)

        self.complete_button = QPushButton("Complete", self)
        self.complete_button.clicked.connect(self.on_complete_button_clicked)
        self.complete_button.setVisible(False)
        self.layout.addWidget(self.complete_button)

        self.emergency_stop_button = QPushButton("Emergency Stop", self)
        self.emergency_stop_button.clicked.connect(self.on_emergency_stop_clicked)
        self.layout.addWidget(self.emergency_stop_button)

        self.restart_button = QPushButton("Restart", self)
        self.restart_button.clicked.connect(self.on_restart_clicked)
        self.restart_button.setVisible(False)
        self.layout.addWidget(self.restart_button)

        # Set layout
        self.setLayout(self.layout)

        # Connect the update signal to the update_status method
        self.update_status_signal.connect(self.update_status)
        self.enable_next_goal_signal.connect(self.enable_next_goal)
        self.emergency_stop_signal.connect(self.handle_emergency_stop)
        self.restart_signal.connect(self.handle_restart)

    def set_background_image(self, image_path):
        """Set the background image of the window."""
        self.background_label.setPixmap(QPixmap(image_path))

    def update_status(self, status):
        if status == "waiting":
            self.status_label.setText("대기 중입니다.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/9.png")
            self.complete_button.setVisible(False)
            self.restart_button.setVisible(False)
        elif status.startswith("deliver"):
            self.status_label.setText("서빙 중입니다.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/9.png")
            self.complete_button.setVisible(False)
            self.restart_button.setVisible(False)
        elif status == "at_goal_start":
            self.status_label.setText("목표 도착: 수령 완료 버튼을 누르세요.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/11.png")
            self.complete_button.setText("수령 완료")
            self.complete_button.setVisible(True)
        elif status.startswith("collect"):
            self.status_label.setText("수거 중입니다.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/9.png")
            self.complete_button.setVisible(False)
            self.restart_button.setVisible(False)
        elif status == "at_goal_return":
            self.status_label.setText("목표 도착: 수거 완료 버튼을 누르세요.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/11.png")
            self.complete_button.setText("수거 완료")
            self.complete_button.setVisible(True)
        elif status == "returning_home":
            self.status_label.setText("복귀 중입니다.")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/9.png")
            self.complete_button.setVisible(False)
            self.restart_button.setVisible(False)
        elif status == "emergency_stop":
            self.status_label.setText("강제 정지됨")
            self.set_background_image("/home/namsang/serving_robot/src/final_system/resource/image/10.png")
            self.complete_button.setVisible(False)
            self.restart_button.setVisible(True)

    def on_complete_button_clicked(self):
        self.complete_button.setVisible(False)
        self.enable_next_goal_signal.emit()

    def on_emergency_stop_clicked(self):
        self.status_label.setText("강제 정지됨")
        self.complete_button.setVisible(False)
        self.restart_button.setVisible(True)
        self.emergency_stop_signal.emit()

    def on_restart_clicked(self):
        self.restart_signal.emit()

    def enable_next_goal(self):
        pass

    def handle_emergency_stop(self):
        pass

    def handle_restart(self):
        pass


class RobotStatusNode(Node, QThread):
    status_signal = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'robot_status_node')
        QThread.__init__(self)
        self.status_subscription = self.create_subscription(
            String,
            'task_waypoints_topic',
            self.status_callback,
            10
        )
        self.get_logger().info("Subscribed to task_waypoints_topic")

        # 액션 클라이언트 생성
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # 테이블 좌표 정의
        self.table_coordinates = {
            0: (0.0, 0.0, 0.0),
            1: (1.0, -0.95, math.pi / 2),
            2: (2.0, -0.95, math.pi / 2),
            3: (3.0, -0.95, math.pi / 2),
            4: (1.0, 0.25, math.pi / 2),
            5: (2.0, 0.25, math.pi / 2),
            6: (3.0, 0.25, math.pi / 2),
            7: (1.0, 1.35, math.pi / 2),
            8: (2.0, 1.35, math.pi / 2),
            9: (3.0, 1.35, math.pi / 2),
            10: (0.3, 2.0, math.pi),  # Return 명령의 특별 좌표
        }

        self.pose_sequence = []
        self.goal_index = 0
        self.emergency_stopped = False
        self.current_command = None

        # 서비스 생성
        self.callback_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            Trigger, 'robot_status_service', self.handle_robot_status_service_request,
            callback_group=self.callback_group
        )

    def run(self):
        rclpy.spin(self)

    def status_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            if len(command_data) < 2:
                self.get_logger().warn("Received incomplete command data.")
                return
            command = command_data[0].lower()
            table_list = command_data[1:]
            self.get_logger().info(f"Received status: {command_data}")
            self.status_signal.emit(command)
            self.current_command = command

            if self.emergency_stopped:
                return

            if command == "deliver":
                waypoints = [self.table_coordinates[table] for table in table_list if table in self.table_coordinates]
                waypoints.append(self.table_coordinates[0])  # 마지막에 대기 위치 추가
                self.get_logger().info(f"Command received: deliver to tables {table_list}")
                self.create_pose_sequence(waypoints)
                self.goal_index = 0
                self.send_next_goal()

            elif command == "waiting":
                self.get_logger().info("Command received: waiting")
                self.create_pose_sequence([self.table_coordinates[0]])
                self.goal_index = 0
                self.send_next_goal()

            elif command == "emergency":
                self.get_logger().info("Emergency command received via waypoint topic. Triggering emergency stop.")
                self.handle_emergency_stop()

            elif command == "collect":
                waypoints = [self.table_coordinates[table] for table in table_list if table in self.table_coordinates]
                waypoints.append(self.table_coordinates[10])  # 마지막에 반환 위치 추가 (수거 위치)
                waypoints.append(self.table_coordinates[0])   # 최종 대기 위치로 이동 (초기 위치)
                self.get_logger().info(f"Command received: collect from tables {table_list}")
                self.create_pose_sequence(waypoints)
                self.goal_index = 0
                self.send_next_goal()
            else:
                self.get_logger().warn("Unrecognized command. Valid commands: 'deliver', 'waiting', 'collect'")
        except (json.JSONDecodeError, IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing command: {e}")

    def create_pose_sequence(self, waypoints):
        self.pose_sequence = []
        for waypoint in waypoints:
            x, y, yaw = waypoint
            quaternion = quaternion_from_euler(0, 0, yaw)
            orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=orientation)
            self.pose_sequence.append(pose)

    def send_next_goal(self):
        if self.emergency_stopped:
            return

        if self.goal_index < len(self.pose_sequence):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose = self.pose_sequence[self.goal_index]

            self.get_logger().info(f"Sending goal pose {self.goal_index + 1} to Action Server")
            if self.goal_index == len(self.pose_sequence) - 1:
                self.status_signal.emit("returning_home")
            future = self.client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback)
            future.add_done_callback(self.goal_done_callback)
        else:
            self.get_logger().info("Sequence complete. Waiting for next command.")
            self.pose_sequence = []
            self.goal_index = 0
            self.status_signal.emit("waiting")  # Send waiting status when back at initial position

    def goal_feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback for goal pose {self.goal_index + 1} received")
        if self.current_command == "deliver":
            self.status_signal.emit("deliver")
        elif self.current_command == "collect":
            self.status_signal.emit("collect")

    def goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal pose {self.goal_index + 1} was rejected")
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(f"Goal pose {self.goal_index + 1} accepted")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal pose {self.goal_index + 1} reached successfully")
            if self.goal_index < len(self.pose_sequence) - 1:
                if self.current_command == "deliver":
                    self.status_signal.emit("at_goal_start")
                elif self.current_command == "collect":
                    self.status_signal.emit("at_goal_return")
            else:
                self.status_signal.emit("waiting")
            self.goal_index += 1
        elif status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            self.get_logger().error(f"Goal pose {self.goal_index + 1} was not successful")

    def handle_emergency_stop(self):
        self.get_logger().info("Emergency stop triggered. Cancelling all goals.")
        self.emergency_stopped = True
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal()
            self.get_logger().info("Current goal cancelled.")

    def handle_restart(self):
        self.get_logger().info("Restarting robot. Returning to initial position.")
        self.emergency_stopped = False
        self.create_pose_sequence([self.table_coordinates[0]])
        self.goal_index = 0
        self.send_next_goal()

    def handle_robot_status_service_request(self, request, response):
        # Handle emergency stop scenario first
        if self.emergency_stopped:
            response.success = True
            response.message = "3"  # Status code for emergency stop
        elif self.current_command == "deliver":
            response.success = True
            response.message = "0"  # Status code for delivering
        elif self.current_command == "collect":
            response.success = True
            response.message = "1"  # Status code for collecting
        elif self.current_command == "waiting" or not self.current_command:
            response.success = True
            response.message = "2"  # Status code for waiting
        else:
            response.success = False
            response.message = "Unknown status"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = RobotStatusGUI()
    gui.show()

    robot_status_node = RobotStatusNode()
    robot_status_node.status_signal.connect(gui.update_status_signal)
    gui.enable_next_goal_signal.connect(robot_status_node.send_next_goal)
    gui.emergency_stop_signal.connect(robot_status_node.handle_emergency_stop)
    gui.restart_signal.connect(robot_status_node.handle_restart)
    robot_status_node.start()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        robot_status_node.get_logger().info("Shutting down GUI and ROS2 nodes.")
    finally:
        robot_status_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
