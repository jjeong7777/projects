import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from gui_1206_1400 import Ui_MainWindow
import cv2
import numpy as np
import serial
import serial.tools.list_ports
from std_msgs.msg import String

class RosCameraThread(QThread):
    """Thread to handle ROS 2 camera topic subscription."""
    image_signal = pyqtSignal(QImage)  # Signal must be declared at the class level

    def __init__(self, camera_topic, node_name):
        super().__init__()
        self.camera_topic = camera_topic
        self.node_name = node_name
        self.bridge = CvBridge()
        self.node = None
        self.running = False

    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(self.node_name)
            self.node.create_subscription(
                CompressedImage,
                self.camera_topic,
                self.callback,
                10
            )
            self.running = True
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"Error in RosCameraThread ({self.node_name}): {e}")
        finally:
            if self.node:
                self.node.destroy_node()

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_signal.emit(qt_image)
        except Exception as e:
            print(f"Error processing image in callback: {e}")

    def stop(self):
        self.running = False
        self.wait()

class USBChecker(QThread):
    """Thread to check USB connection status."""
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, ros_node):
        super().__init__()
        self.running = True
        self.last_checked_port = None
        self.ros_node = ros_node

    def run(self):
        while self.running:
            current_port = self.find_arduino_port()
            if current_port != self.last_checked_port:
                self.last_checked_port = current_port
                is_connected = current_port is not None
                self.connection_status_changed.emit(is_connected)
                if not is_connected:
                    self.ros_node.publish_error_message("conveyor belt error")
            self.msleep(5000)

    def stop(self):
        self.running = False
        self.wait()

    def find_arduino_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or "ttyACM" in port.device:
                return port.device
        return None

class ROSNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        self.publisher_ = self.create_publisher(String, 'usb_status', 10)

    def publish_error_message(self, error_message):
        msg = String()
        msg.data = error_message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published error message: {error_message}")

class MainApp(QMainWindow):
    """Main application GUI."""
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_ui()

        # ROS Node (USB checker)
        self.ros_node = ROSNode()

        # Initialize ROS Camera Threads
        self.global_camera_thread = RosCameraThread(camera_topic='/camera/image', node_name='global_camera_node')
        self.global_camera_thread.image_signal.connect(self.update_global_camera_feed)
        self.global_camera_thread.start()

        self.robot_camera_thread = RosCameraThread(camera_topic='/robot/image', node_name='robot_camera_node')
        self.robot_camera_thread.image_signal.connect(self.update_robot_camera_feed)
        self.robot_camera_thread.start()

        # USB checker initialization
        self.arduino = None
        self.usb_checker = USBChecker(self.ros_node)
        self.usb_checker.connection_status_changed.connect(self.handle_connection_status)
        self.usb_checker.start()

    def init_ui(self):
        self.ui.login_button.clicked.connect(self.verify_login)
        self.ui.emegency_stop_btn.clicked.connect(self.emergency_stop)
        self.ui.train_data_btn.clicked.connect(self.train_data)
        self.ui.btn_goal_step.clicked.connect(self.send_step_to_arduino)
        self.ui.btn_goal_range.clicked.connect(self.send_range_to_arduino)

    def handle_connection_status(self, connected):
        if connected:
            try:
                self.arduino = serial.Serial(port=self.usb_checker.last_checked_port, baudrate=115200, timeout=1)
                self.update_conveyor_connection_state(True)
                print(f"Reconnected to Arduino on {self.usb_checker.last_checked_port}")
            except serial.SerialException as e:
                print(f"Error reconnecting to Arduino: {e}")
                self.update_conveyor_connection_state(False)
        else:
            if self.arduino and self.arduino.is_open:
                self.arduino.close()
            self.arduino = None
            self.update_conveyor_connection_state(False)
            print("Arduino disconnected.")

    def update_conveyor_connection_state(self, connected):
        if connected:
            self.ui.conveyor_connection_state.setStyleSheet("background-color: green; color: white;")
            self.ui.conveyor_connection_state.setText("Connection!")
        else:
            self.ui.conveyor_connection_state.setStyleSheet("background-color: red; color: white;")
            self.ui.conveyor_connection_state.setText("Disconnect!")

    def send_step_to_arduino(self):
        if self.arduino and self.arduino.is_open:
            step_value = self.ui.label_goal_step2.text()
            if step_value.isdigit():
                try:
                    self.arduino.write(f"{step_value}\n".encode())
                    print(f"Sent to Arduino: {step_value}")
                except serial.SerialException as e:
                    print(f"Error sending data to Arduino: {e}")
            else:
                QMessageBox.warning(self, "Input Error", "Please enter a valid number!")
        else:
            QMessageBox.warning(self, "Arduino Error", "Arduino is not connected!")

    def send_range_to_arduino(self):
        if self.arduino and self.arduino.is_open:
            range_value = 9795
            try:
                self.arduino.write(f"{range_value}\n".encode())
                print(f"Sent to Arduino: {range_value}")
            except serial.SerialException as e:
                print(f"Error sending data to Arduino: {e}")
        else:
            QMessageBox.warning(self, "Arduino Error", "Arduino is not connected!")

    def verify_login(self):
        valid_id = "admin"
        valid_password = "1234"
        entered_id = self.ui.id_line_edit.text()
        entered_password = self.ui.password_line_edit.text()
        if entered_id == valid_id and entered_password == valid_password:
            self.ui.main_widget.setCurrentIndex(1)
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid ID or Password!")

    def emergency_stop(self):
        print("Emergency Stop button clicked!")

    def train_data(self):
        print("Train Data button clicked!")

    def update_global_camera_feed(self, qt_image):
        """Update global camera feed in the UI."""
        pixmap = QPixmap.fromImage(qt_image)
        if self.ui.global_camera:
            self.ui.global_camera.setPixmap(pixmap)

    def update_robot_camera_feed(self, qt_image):
        """Update robot camera feed in the UI."""
        pixmap = QPixmap.fromImage(qt_image)
        if self.ui.robot_camera:
            self.ui.robot_camera.setPixmap(pixmap)

    def closeEvent(self, event):
        # Ensure all threads are stopped and resources are cleaned up
        self.global_camera_thread.stop()
        self.robot_camera_thread.stop()
        self.usb_checker.stop()
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == "__main__":s
    rclpy.init()
    app = QApplication(sys.argv)
    main_window = MainApp()
    main_window.show()
    sys.exit(app.exec_())
