from PySide2.QtCore import *
from PySide2.QtWidgets import *
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import os
from PySide2.QtGui import QFont, QPixmap, QPalette, QBrush
from functools import partial
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS 메시지를 발행하는 노드 클래스
class TableOrderPublisher(Node):


    def __init__(self):
        super().__init__('table_node')

        # QoS 프로파일 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.order_publisher = self.create_publisher(String, 'table_order_topic', qos_profile)
        self.table_id = 1

    def set_table_id(self, table_id):
        self.table_id = table_id

    def publish_order(self, order_details):
        if not rclpy.ok():
            self.get_logger().error('Cannot publish, rclpy is not running.')
            return
        order_details['Table_id'] = self.table_id
        msg = String()
        msg.data = json.dumps(order_details, ensure_ascii=False)
        self.order_publisher.publish(msg)
        self.get_logger().info(f'Published order: {msg.data}')
        
    def set_main_window(self, main_window):
        self.main_window = main_window
        
# ROS 노드를 백그라운드에서 실행하는 클래스
class ROSWorker(QRunnable):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        finally:
            executor.shutdown()

# 시작 창 클래스
class StartWindow(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.setWindowTitle("Start Window")
        self.setGeometry(100, 100, 1024, 768)

        # ROS 노드 설정
        if ros_node is None:
            rclpy.init(args=None)
            self.ros_node = TableOrderPublisher()
            self.ros_node.set_main_window(self)
        else:
            self.ros_node = ros_node

        # ROS 스레드 실행을 위한 스레드 풀 설정
        self.thread_pool = QThreadPool()
        self.ros_worker = ROSWorker(self.ros_node)
        self.thread_pool.start(self.ros_worker)

        self.setupUi()

    # UI 설정 함수
    def setupUi(self):
        # 배경 이미지 설정
        image_path = "/home/namsang/final/background.png"
        if os.path.exists(image_path):
            palette = QPalette()
            pixmap = QPixmap(image_path).scaled(self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
            palette.setBrush(QPalette.Window, QBrush(pixmap))
            self.setAutoFillBackground(True)
            self.setPalette(palette)

        self.start_layout = QVBoxLayout(self)
        self.start_layout.setAlignment(Qt.AlignCenter)

        # 안내 레이블 설정
        self.start_label = QLabel("주문을 시작하시려면 주문하기 버튼을 눌러주세요")
        self.start_label.setAlignment(Qt.AlignCenter)
        self.start_label.setStyleSheet("color: black; font-size: 20px; background-color: rgba(255, 255, 255, 180);")
        self.start_layout.addWidget(self.start_label)

        # 주문하기 버튼 설정
        self.start_order_button = QPushButton("주문하기")
        self.start_order_button.setFixedHeight(70)
        self.start_order_button.setFixedWidth(200)
        self.start_order_button.setStyleSheet("font-size: 20px; font-weight: bold; background-color: white;")
        self.start_order_button.clicked.connect(self.open_main_window)
        self.start_layout.addWidget(self.start_order_button, alignment=Qt.AlignCenter)

    # 메인 윈도우를 여는 함수
    def open_main_window(self):
        self.main_window = Ui_MainWindow(self.ros_node, self.thread_pool)
        self.main_window.show()
        self.close()

# 메인 윈도우 클래스
class Ui_MainWindow(QMainWindow):
    def __init__(self, ros_node, thread_pool):
        super().__init__()
        self.ros_node = ros_node
        self.thread_pool = thread_pool
        self.setupUi()

    # 시작 창으로 돌아가는 함수
    def go_back_to_start(self):
        self.start_window = StartWindow(self.ros_node)
        self.start_window.show()
        self.close()

    # UI 설정 함수
    def setupUi(self):
        self.setWindowTitle("MainWindow")
        self.resize(1024, 768)

        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        self.main_layout = QVBoxLayout(self.centralwidget)

        # 상단 프레임 설정
        self.top_frame = QFrame(self.centralwidget)
        self.top_frame.setStyleSheet("background-color: lightgray; border: 2px solid black;")
        self.top_frame.setFixedHeight(50)

        self.top_label = QLabel(f"테이블 번호: {self.ros_node.table_id}", self.top_frame)
        self.top_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.top_label.setAlignment(Qt.AlignCenter)

        self.back_button = QPushButton('<-', self.top_frame)
        self.back_button.setFixedSize(30, 30)
        self.back_button.setStyleSheet("background-color: white; border: 2px solid black;")
        self.back_button.clicked.connect(self.go_back_to_start)

        self.table_id_spinbox = QSpinBox(self.top_frame)
        self.table_id_spinbox.setRange(1, 100)
        self.table_id_spinbox.setValue(self.ros_node.table_id)
        self.table_id_spinbox.valueChanged.connect(self.update_table_id)

        top_layout = QHBoxLayout(self.top_frame)
        top_layout.addWidget(self.top_label, alignment=Qt.AlignCenter)
        top_layout.addStretch()
        top_layout.addWidget(QLabel("테이블 번호 변경: "))
        top_layout.addWidget(self.table_id_spinbox)
        top_layout.addWidget(self.back_button, alignment=Qt.AlignRight)

        self.main_layout.addWidget(self.top_frame)
        self.content_layout = QHBoxLayout()

        # 사이드바 설정
        self.sidebar = QFrame(self.centralwidget)
        self.sidebar.setStyleSheet("background-color: lightgray; border: 2px solid black;")
        self.sidebar_layout = QVBoxLayout(self.sidebar)

        self.drinks_btn = QPushButton("술")
        self.food_btn = QPushButton("음식")
        self.call_btn = QPushButton("서비스 요청")
        self.other_table_btn = QPushButton("다른 테이블")

        for btn in [self.drinks_btn, self.food_btn, self.call_btn, self.other_table_btn]:
            btn.setFixedHeight(100)
            btn.setStyleSheet("background-color: white;")
            self.sidebar_layout.addWidget(btn)

        self.stacked_widget = QStackedWidget(self.centralwidget)
        self.stacked_widget.setStyleSheet("background-color: white; border: 2px solid black;")

        self.order_summary_sidebar = QFrame(self.centralwidget)
        self.order_summary_sidebar.setStyleSheet("background-color: lightgray; border: 2px solid black;")
        self.order_summary_sidebar.setFixedWidth(300)
        self.order_summary_layout = QVBoxLayout(self.order_summary_sidebar)

        # 주문 목록 설정
        self.order_summary_title = QLabel("주문 목록")
        self.order_summary_title.setAlignment(Qt.AlignCenter)
        self.order_summary_layout.addWidget(self.order_summary_title)

        self.order_summary_scroll_area = QScrollArea()
        self.order_summary_scroll_area.setWidgetResizable(True)
        self.order_summary_widget = QWidget()
        self.order_summary_scroll_layout = QVBoxLayout(self.order_summary_widget)
        self.order_summary_scroll_area.setWidget(self.order_summary_widget)
        self.order_summary_layout.addWidget(self.order_summary_scroll_area)

        self.total_price_label = QLabel("총 합계: 0원")
        self.total_price_label.setAlignment(Qt.AlignCenter)
        self.order_summary_layout.addWidget(self.total_price_label)

        self.order_button = QPushButton("주문하기")
        self.order_button.setStyleSheet("background-color: white;")
        self.order_button.clicked.connect(self.handle_order)
        self.order_summary_layout.addWidget(self.order_button)

        # 각 페이지 설정 함수 호출
        self.setup_drink_page()
        self.setup_food_page()
        self.setup_call_page()
        self.setup_other_table_page()

        self.content_layout.addWidget(self.sidebar)
        self.content_layout.addWidget(self.stacked_widget)
        self.content_layout.addWidget(self.order_summary_sidebar)

        self.main_layout.addWidget(self.top_frame)
        self.main_layout.addLayout(self.content_layout)

        self.drinks_btn.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))
        self.food_btn.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        self.call_btn.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))
        self.other_table_btn.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(3))

        self.total_price = 0
        self.order_summary = {}

    # 테이블 번호 업데이트 함수
    def update_table_id(self, value):
        self.ros_node.set_table_id(value)
        self.top_label.setText(f"테이블 번호: {value}")

    # 각 페이지의 UI 설정 함수들 (음료, 음식, 서비스 요청 등)
    def setup_drink_page(self):
        self.drink_page = QWidget()
        self.drink_layout = QGridLayout(self.drink_page)
        self.drink_page.setStyleSheet("background-color: lightblue;")

        drink_names = ["소주", "맥주", "막걸리", "와인", "하이볼", "사케"]
        drink_prices = [5000, 4000, 6000, 15000, 12000, 10000]
        drink_images = [
            "/home/namsang/serving_robot/src/final_system/resource/image/soju.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/beer.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/막걸리.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/wine.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/하이볼.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/sake.png"
        ]

        for i in range(6):
            self.add_item_to_layout(self.drink_layout, drink_names[i], drink_prices[i], drink_images[i], i)

        self.stacked_widget.addWidget(self.drink_page)

    def setup_food_page(self):
        self.food_page = QWidget()
        self.food_layout = QGridLayout(self.food_page)
        self.food_page.setStyleSheet("background-color: lightgreen;")

        food_names = ["소시지&감튀", "오뎅탕", "곱창전골", "오돌뼈", "골벵이무침", "모둠과일"]
        food_prices = [3000, 5000, 4000, 4500, 7000, 6000]
        food_images = [
            "/home/namsang/serving_robot/src/final_system/resource/image/술안주6_소시지&감튀.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/안주1_오뎅탕.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/안주2_곱창전골.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/안주3_오돌뼈.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/안주4_골뱅이무침.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/안주5_모둠과.png"
        ]

        for i in range(6):
            self.add_item_to_layout(self.food_layout, food_names[i], food_prices[i], food_images[i], i)

        self.stacked_widget.addWidget(self.food_page)

    def setup_call_page(self):
        self.call_page = QWidget()
        self.call_layout = QGridLayout(self.call_page)
        self.call_page.setStyleSheet("background-color: lightyellow;")

        call_services = ["물 요청", "냅킨 요청", "메뉴판 요청", "직원 호출"]
        service_prices = [0, 0, 0, 0]
        service_images = [
            "/home/namsang/serving_robot/src/final_system/resource/image/물.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/냅킨.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/메뉴판.png",
            "/home/namsang/serving_robot/src/final_system/resource/image/직원호출.png"
        ]

        for i in range(4):
            self.add_item_to_layout(self.call_layout, call_services[i], service_prices[i], service_images[i], i)

        self.stacked_widget.addWidget(self.call_page)

    def setup_other_table_page(self):
        self.other_table_page = QWidget()
        self.other_table_layout = QGridLayout(self.other_table_page)
        self.other_table_page.setStyleSheet("background-color: lightcoral;")

        other_table_services = ["테이블 2 서비스", "테이블 3 서비스", "테이블 4 서비스", "테이블 5 서비스"]
        other_table_prices = [0, 0, 0, 0]
        other_table_images = [
            "images/table2.png",
            "images/table3.png",
            "images/table4.png",
            "images/table5.png"
        ]

        for i in range(4):
            self.add_item_to_layout(self.other_table_layout, other_table_services[i], other_table_prices[i], other_table_images[i], i)

        self.stacked_widget.addWidget(self.other_table_page)

    # 주문 처리 함수
    def handle_order(self):
        if not self.order_summary:
            QMessageBox.information(self, "주문하기", "주문 목록이 비어 있습니다.")
        else:
            # 주문 요약을 JSON으로 변환하여 ROS 메시지로 발행
            order_details_list = []
            for item_name, details in self.order_summary.items():
                item_details = {
                    "Item": item_name,
                    "Quantity": details['quantity'],
                    "Price": details['total_price']
                }
                order_details_list.append(item_details)

            order_summary_json = {
                "order": order_details_list,
                "Amount": self.total_price,
                "Table_id": self.ros_node.table_id
            }
            self.ros_node.publish_order(order_summary_json)

            QMessageBox.information(self, "주문 완료", "주문이 완료되었습니다.")
            self.order_summary = {}
            self.update_order_summary()

    def add_item_to_layout(self, layout, item_name, item_price, item_image, index):
        item_frame = QFrame()
        item_frame.setStyleSheet("background-color: white; border: 2px solid black;")
        item_layout = QVBoxLayout(item_frame)

        menu_name_label = QLabel(item_name)
        menu_name_label.setAlignment(Qt.AlignCenter)

        image_label = QLabel()
        image_label.setFixedSize(150, 150)
        if os.path.exists(item_image):
            pixmap = QPixmap(item_image)
            image_label.setPixmap(pixmap.scaled(image_label.width(), image_label.height(), Qt.KeepAspectRatio))
        else:
            image_label.setText("이미지 없음")
        image_label.setAlignment(Qt.AlignCenter)

        price_label = QLabel(f"가격: {item_price}원")
        price_label.setAlignment(Qt.AlignCenter)

        btn_layout = QHBoxLayout()
        btn_minus = QPushButton("-")
        btn_minus.setFixedWidth(40)
        btn_minus.setStyleSheet("background-color: white;")
        btn_plus = QPushButton("+")
        btn_plus.setFixedWidth(40)
        btn_plus.setStyleSheet("background-color: white;")
        quantity_label = QLineEdit("0")
        quantity_label.setFixedWidth(40)
        quantity_label.setAlignment(Qt.AlignCenter)
        quantity_label.setReadOnly(True)

        btn_minus.clicked.connect(lambda: self.decrease_quantity(quantity_label))
        btn_plus.clicked.connect(lambda: self.increase_quantity(quantity_label))

        btn_layout.addWidget(btn_minus)
        btn_layout.addWidget(quantity_label)
        btn_layout.addWidget(btn_plus)

        order_btn = QPushButton("주문하기")
        order_btn.setStyleSheet("background-color: white;")
        order_btn.clicked.connect(lambda: self.add_to_order(item_name, item_price, quantity_label))

        item_layout.addWidget(menu_name_label)
        item_layout.addWidget(image_label)
        item_layout.addWidget(price_label)
        item_layout.addLayout(btn_layout)
        item_layout.addWidget(order_btn)

        layout.addWidget(item_frame, index // 3, index % 3)

    def decrease_quantity(self, quantity_label):
        current_value = int(quantity_label.text())
        if current_value > 0:
            new_value = current_value - 1
            quantity_label.setText(str(new_value))

    def increase_quantity(self, quantity_label):
        current_value = int(quantity_label.text())
        if current_value < 100:
            new_value = current_value + 1
            quantity_label.setText(str(new_value))

    def add_to_order(self, item_name, item_price, quantity_label):
        quantity = int(quantity_label.text())
        if quantity > 0:
            if item_name in self.order_summary:
                self.order_summary[item_name]['quantity'] += quantity
                self.order_summary[item_name]['total_price'] += quantity * item_price
            else:
                self.order_summary[item_name] = {
                    'quantity': quantity,
                    'total_price': quantity * item_price
                }
            self.update_order_summary()
            quantity_label.setText("0")

    # 주문 목록 업데이트 함수
    def update_order_summary(self):
        for i in reversed(range(self.order_summary_scroll_layout.count())):
            widget = self.order_summary_scroll_layout.itemAt(i).widget()
            if widget is not None:
                widget.setParent(None)

        self.total_price = 0
        for item_name, details in self.order_summary.items():
            quantity = details['quantity']
            total_price = details['total_price']

            order_item_frame = QFrame()
            order_item_frame.setStyleSheet("background-color: #f0f0f0; border-bottom: 1px solid #d3d3d3;")
            order_item_layout = QHBoxLayout(order_item_frame)

            order_label = QLabel(f"{item_name} x {quantity} = {total_price}원")
            delete_btn = QPushButton("삭제")
            delete_btn.setFixedSize(50, 30)
            delete_btn.setStyleSheet("background-color: white;")
            delete_btn.clicked.connect(partial(self.remove_order_item, item_name))

            order_item_layout.addWidget(order_label)
            order_item_layout.addWidget(delete_btn)
            order_item_layout.setAlignment(Qt.AlignTop)

            self.order_summary_scroll_layout.addWidget(order_item_frame)

            self.total_price += total_price

        self.total_price_label.setText(f"총 합계: {self.total_price}원")

    def remove_order_item(self, item_name):
        if item_name in self.order_summary:
            del self.order_summary[item_name]
            self.update_order_summary()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    start_window = StartWindow()
    start_window.show()
    app.aboutToQuit.connect(rclpy.shutdown)
    sys.exit(app.exec_())
