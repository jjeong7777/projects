import sqlite3
import sys
import json
import datetime
from threading import Thread
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QGroupBox,
    QHBoxLayout, QMessageBox, QScrollArea, QFrame, QDialog, QTableWidget, QTableWidgetItem,
    QTabWidget
)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # 적절한 서비스 타입 사용
from rclpy.task import Future
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class ROSNodeWorker(QThread):
    table_order_trigger = pyqtSignal(dict)
    delivered_list_trigger = pyqtSignal(str)
    robot_status_trigger = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = NODE()
        self.node.table_order_received = self.table_order_trigger
        self.node.delivered_list_received = self.delivered_list_trigger
        self.node.robot_status_received = self.robot_status_trigger

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


class NODE(Node):
    def __init__(self, node_name='ros_gui_node'):
        super().__init__(node_name)
        self.table_order_received = None
        self.delivering_list = []
        self.collecting_list = set()
        self.create_subscription(String, 'table_order_topic', self.subscription_callback, 10)
        self.task_waypoints_pub = self.create_publisher(String, 'task_waypoints_topic', 10)
        self.robot_status_client = self.create_client(Trigger, 'robot_status_service')  # 서비스 타입 변경
        self.init_db()

        # Start a timer to call the robot status service every second
        self.timer = self.create_timer(1.0, self.call_robot_status_service)

    def init_db(self):
        today = datetime.datetime.now().strftime('%Y%m%d')
        db_name = f'order_table_{today}.db'
        self.conn = sqlite3.connect(db_name)
        self.cursor = self.conn.cursor()
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS orders (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                Timestamp TEXT,
                Table_id TEXT,
                Item TEXT,
                Quantity INTEGER,
                Amount REAL,
                Status TEXT
            )
        """)
        # 추가 정보 테이블 생성
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS sales_summary (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                Item TEXT,
                Total_Quantity INTEGER,
                Total_Amount REAL
            )
        """)
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS best_selling (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                Best_Item TEXT,
                Total_Revenue REAL
            )
        """)
        self.conn.commit()

    def save_order_info(self, table_id, item, quantity, price, amount, status):
        try:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            today = datetime.datetime.now().strftime('%Y%m%d')
            db_name = f'order_table_{today}.db'
            with sqlite3.connect(db_name) as conn:
                cursor = conn.cursor()
                cursor.execute('''
                    INSERT INTO orders (Timestamp, Table_id, Item, Quantity, Amount, Status)
                    VALUES (?, ?, ?, ?, ?, ?)
                ''', (timestamp, table_id, item, quantity, amount, status))
                
                # 판매 요약 업데이트
                cursor.execute('''
                    SELECT Total_Quantity, Total_Amount FROM sales_summary WHERE Item = ?
                ''', (item,))
                row = cursor.fetchone()
                if row:
                    total_quantity = row[0] + quantity
                    total_amount = row[1] + amount
                    cursor.execute('''
                        UPDATE sales_summary SET Total_Quantity = ?, Total_Amount = ? WHERE Item = ?
                    ''', (total_quantity, total_amount, item))
                else:
                    cursor.execute('''
                        INSERT INTO sales_summary (Item, Total_Quantity, Total_Amount)
                        VALUES (?, ?, ?)
                    ''', (item, quantity, amount))
                
                # 베스트셀러 및 총 수익 업데이트
                cursor.execute('''
                    SELECT Item, Total_Quantity, Total_Amount FROM sales_summary ORDER BY Total_Quantity DESC LIMIT 1
                ''')
                best_selling_item = cursor.fetchone()
                if best_selling_item:
                    cursor.execute('''
                        INSERT OR REPLACE INTO best_selling (id, Best_Item, Total_Revenue)
                        VALUES (1, ?, ?)
                    ''', (best_selling_item[0], best_selling_item[2]))
                
                conn.commit()
        except sqlite3.Error as e:
            print(f"Failed to save order to database: {e}")

    def subscription_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received JSON: {data}")
            if self.table_order_received:
                self.table_order_received.emit(data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON message: {e}")

    def call_robot_status_service(self):
        if not self.robot_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Robot status service not available')
            return

        request = Trigger.Request()  # Trigger 타입 요청 생성
        future = self.robot_status_client.call_async(request)
        future.add_done_callback(self.handle_robot_status_response)

    def handle_robot_status_response(self, future: Future):
        try:
            response = future.result()
            if self.robot_status_received:
                self.robot_status_received.emit(response.message)
        except Exception as e:
            self.get_logger().error(f'Failed to call robot status service: {e}')


class GUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("음식점 주문 시스템")
        self.setGeometry(100, 100, 1400, 800)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        self.customer_order_info = QLabel("고객 주문 정보")
        self.customer_order_info.setStyleSheet(
            "background-color: #0078d7; color: white; font-size: 16px; font-weight: bold; padding: 10px;"
        )
        self.main_layout.addWidget(self.customer_order_info)
        self.bottom_layout = QHBoxLayout()
        self.main_layout.addLayout(self.bottom_layout)
        self.scroll_area = QScrollArea()
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_widget = QWidget()
        self.scroll_layout = QHBoxLayout(self.scroll_widget)
        self.scroll_layout.setSpacing(10)
        self.scroll_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setWidget(self.scroll_widget)
        self.bottom_layout.addWidget(self.scroll_area)
        self.robot_controls_layout = QVBoxLayout()
        self.init_robot_controls()
        self.bottom_layout.addLayout(self.robot_controls_layout)
        self.delivering_list = []
        self.collecting_list = set()
        self.accepted_orders = set()
        self.start_button = None

    def init_robot_controls(self):
        self.robot_status_label = QLabel("로봇 상태: 알 수 없음")
        self.robot_status_label.setAlignment(Qt.AlignCenter)
        self.robot_status_label.setStyleSheet("background-color: gray; color: white; font-size: 16px;")
        self.robot_controls_layout.addWidget(self.robot_status_label)

        self.call_robot_btn = QPushButton("로봇 호출")
        self.collect_btn = QPushButton("수거하기")
        self.database_btn = QPushButton("데이터베이스 관리")
        self.emergency_stop_btn = QPushButton("긴급 정지")
        self.emergency_stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        button_height = 120
        self.call_robot_btn.setFixedHeight(button_height)
        self.collect_btn.setFixedHeight(button_height)
        self.database_btn.setFixedHeight(button_height)
        self.emergency_stop_btn.setFixedHeight(button_height)
        self.call_robot_btn.clicked.connect(self.show_delivery_page)
        self.collect_btn.clicked.connect(self.show_collect_page)
        self.emergency_stop_btn.clicked.connect(self.handle_emergency_stop)
        self.database_btn.clicked.connect(self.handle_database)

        self.robot_controls_layout.addWidget(self.call_robot_btn)
        self.robot_controls_layout.addWidget(self.collect_btn)
        self.robot_controls_layout.addWidget(self.database_btn)
        self.robot_controls_layout.addWidget(self.emergency_stop_btn)

    def handle_database(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("데이터베이스 관리")
        dialog.setGeometry(150, 150, 800, 600)
        layout = QVBoxLayout()
        tab_widget = QTabWidget()

        # 첫 번째 탭: 주문 데이터
        orders_tab = QWidget()
        orders_layout = QVBoxLayout()
        orders_table = QTableWidget()
        orders_table.setColumnCount(7)
        orders_table.setHorizontalHeaderLabels(['ID', 'Timestamp', 'Table ID', 'Item', 'Quantity', 'Amount', 'Status'])
        try:
            today = datetime.datetime.now().strftime('%Y%m%d')
            db_name = f'order_table_{today}.db'
            with sqlite3.connect(db_name) as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT * FROM orders ORDER BY Timestamp")
                rows = cursor.fetchall()
                orders_table.setRowCount(len(rows))
                for row_idx, row in enumerate(rows):
                    for col_idx, value in enumerate(row):
                        orders_table.setItem(row_idx, col_idx, QTableWidgetItem(str(value)))
                orders_layout.addWidget(orders_table)
        except sqlite3.Error as e:
            error_label = QLabel(f"데이터베이스 로드 실패: {e}")
            orders_layout.addWidget(error_label)
        orders_tab.setLayout(orders_layout)

        # 두 번째 탭: 판매 요약 데이터
        summary_tab = QWidget()
        summary_layout = QVBoxLayout()
        summary_table = QTableWidget()
        summary_table.setColumnCount(4)
        summary_table.setHorizontalHeaderLabels(['ID', 'Item', 'Total Quantity', 'Total Amount'])
        try:
            today = datetime.datetime.now().strftime('%Y%m%d')
            db_name = f'order_table_{today}.db'
            with sqlite3.connect(db_name) as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT * FROM sales_summary ORDER BY Item")
                rows = cursor.fetchall()
                summary_table.setRowCount(len(rows))
                total_revenue = 0
                for row_idx, row in enumerate(rows):
                    for col_idx, value in enumerate(row):
                        summary_table.setItem(row_idx, col_idx, QTableWidgetItem(str(value)))
                    total_revenue += row[2]
                    
                summary_layout.addWidget(summary_table)
                summary_table.setItem(len(rows), 3, QTableWidgetItem("Total Revenue:"))
                summary_table.setItem(len(rows), 3, QTableWidgetItem(str(total_revenue)))
        except sqlite3.Error as e:
            error_label = QLabel(f"데이터베이스 로드 실패: {e}")
            summary_layout.addWidget(error_label)
        summary_tab.setLayout(summary_layout)

        # 세 번째 탭: 베스트셀러 및 총 수익 데이터
        best_selling_tab = QWidget()
        best_selling_layout = QVBoxLayout()
        best_selling_table = QTableWidget()
        best_selling_table.setColumnCount(2)
        best_selling_table.setHorizontalHeaderLabels(['Popular Menu Item', 'Total Revenue'])
        try:
            today = datetime.datetime.now().strftime('%Y%m%d')
            db_name = f'order_table_{today}.db'
            with sqlite3.connect(db_name) as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT Best_Item, Total_Revenue FROM best_selling WHERE id = 1")
                row = cursor.fetchone()
                best_selling_table.setRowCount(1)
                if row:
                    best_selling_table.setItem(0, 0, QTableWidgetItem(str(row[0])))
                    best_selling_table.setItem(0, 1, QTableWidgetItem(str(row[1])))
                best_selling_layout.addWidget(best_selling_table)
        except sqlite3.Error as e:
            error_label = QLabel(f"데이터베이스 로드 실패: {e}")
            best_selling_layout.addWidget(error_label)
        best_selling_tab.setLayout(best_selling_layout)

        # 탭 추가
        tab_widget.addTab(orders_tab, "주문 데이터")
        tab_widget.addTab(summary_tab, "판매 요약")
        tab_widget.addTab(best_selling_tab, "인기메뉴 및 총 수익")

        layout.addWidget(tab_widget)
        dialog.setLayout(layout)
        dialog.exec_()

    def update_robot_status(self, status):
        self.robot_status_label.setText(f"로봇 상태: {status}")
        if status == "0":
            self.robot_status_label.setText("로봇 상태: 서빙 중")
            self.robot_status_label.setStyleSheet("background-color: lightgray; color: black; font-size: 16px;")
        elif status == "1":
            self.robot_status_label.setText("로봇 상태: 수거 중")
            self.robot_status_label.setStyleSheet("background-color: lightblue; color: black; font-size: 16px;")
        elif status == "2":
            self.robot_status_label.setText("로봇 상태: 대기 중")
            self.robot_status_label.setStyleSheet("background-color: lightgreen; color: black; font-size: 16px;")
        elif status == "3":
            self.robot_status_label.setText("로봇 상태: 긴급 정지")
            self.robot_status_label.setStyleSheet("background-color: red; color: white; font-size: 16px;")
        else:
            self.robot_status_label.setText("로봇 상태: 알 수 없음")
            self.robot_status_label.setStyleSheet("background-color: gray; color: white; font-size: 16px;")

    def show_delivery_page(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("서빙 페이지")
        dialog.setGeometry(150, 150, 400, 500)
        layout = QVBoxLayout()
        top_frame = QFrame()
        top_layout = QVBoxLayout(top_frame)
        title_label = QLabel("서빙 가능 테이블 목록")
        title_label.setStyleSheet(
            "background-color: blue; color: white; font-size: 18px; padding: 10px;"
        )
        title_label.setAlignment(Qt.AlignCenter)
        top_layout.addWidget(title_label)
        layout.addWidget(top_frame)
        middle_frame = QFrame()
        self.middle_layout = QVBoxLayout(middle_frame)
        self.update_delivery_list()
        layout.addWidget(middle_frame)
        bottom_frame = QFrame()
        bottom_layout = QVBoxLayout(bottom_frame)
        self.start_button = QPushButton("서빙 시작")
        self.start_button.setStyleSheet("font-size: 16px; padding: 10px;")
        self.start_button.setEnabled(bool(self.accepted_orders))
        self.start_button.clicked.connect(lambda: self.get_accepted_delivery_list(dialog))
        bottom_layout.addWidget(self.start_button)
        layout.addWidget(bottom_frame)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_collect_page(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("수거 페이지")
        dialog.setGeometry(150, 150, 400, 500)
        layout = QVBoxLayout()
        top_frame = QFrame()
        top_layout = QVBoxLayout(top_frame)
        title_label = QLabel("수거 가능 테이블 목록")
        title_label.setStyleSheet(
            "background-color: blue; color: white; font-size: 18px; padding: 10px;"
        )
        title_label.setAlignment(Qt.AlignCenter)
        top_layout.addWidget(title_label)
        layout.addWidget(top_frame)
        middle_frame = QFrame()
        self.middle_layout = QVBoxLayout(middle_frame)
        self.update_collection_list()
        layout.addWidget(middle_frame)
        bottom_frame = QFrame()
        bottom_layout = QVBoxLayout(bottom_frame)
        start_button = QPushButton("수거 시작")
        start_button.setStyleSheet("font-size: 16px; padding: 10px;")
        start_button.clicked.connect(lambda: self.get_accepted_collection_list(dialog))
        bottom_layout.addWidget(start_button)
        layout.addWidget(bottom_frame)
        dialog.setLayout(layout)
        dialog.exec_()

    def update_delivery_list(self):
        while self.middle_layout.count():
            widget = self.middle_layout.takeAt(0).widget()
            if widget:
                widget.deleteLater()
        for table in sorted(self.delivering_list):
            item_layout = QHBoxLayout()
            table_label = QLabel(f"테이블 번호: {table}")
            table_label.setStyleSheet("font-size: 16px; padding: 5px;")
            item_layout.addWidget(table_label)
            remove_button = QPushButton("X")
            remove_button.setStyleSheet("color: white; background-color: red; padding: 5px;")
            remove_button.clicked.connect(lambda _, t=table: self.remove_table(t))
            item_layout.addWidget(remove_button)
            container_frame = QFrame()
            container_frame.setLayout(item_layout)
            self.middle_layout.addWidget(container_frame)

    def update_collection_list(self):
        while self.middle_layout.count():
            widget = self.middle_layout.takeAt(0).widget()
            if widget:
                widget.deleteLater()
        for table in sorted(self.collecting_list):
            item_layout = QHBoxLayout()
            table_label = QLabel(f"테이블 번호: {table}")
            table_label.setStyleSheet("font-size: 16px; padding: 5px;")
            item_layout.addWidget(table_label)
            remove_button = QPushButton("X")
            remove_button.setStyleSheet("color: white; background-color: red; padding: 5px;")
            remove_button.clicked.connect(lambda _, t=table: self.remove_collecting_table(t))
            item_layout.addWidget(remove_button)
            container_frame = QFrame()
            container_frame.setLayout(item_layout)
            self.middle_layout.addWidget(container_frame)

    def remove_table(self, table):
        if table in self.delivering_list:
            self.delivering_list.remove(table)
            self.update_delivery_list()

    def remove_collecting_table(self, table):
        if table in self.collecting_list:
            self.collecting_list.remove(table)
            self.update_collection_list()

    def get_accepted_delivery_list(self, dialog):
        self.accepted_delivery_list = ["deliver"] + sorted(self.delivering_list)
        if self.ros_node and self.ros_node.task_waypoints_pub:
            msg = String()
            msg.data = json.dumps(self.accepted_delivery_list)
            self.ros_node.task_waypoints_pub.publish(msg)
        message = QMessageBox(dialog)
        message.setWindowTitle("배달 시작")
        message.setText(", ".join(map(str, self.accepted_delivery_list)))
        message.setStyleSheet("font-size: 16px;")
        message.show()
        QTimer.singleShot(2000, message.close)
        dialog.close()

        if self.start_button:
            self.collecting_list.update(self.delivering_list)
            self.delivering_list.clear()
            self.start_button.setEnabled(False)

    def get_accepted_collection_list(self, dialog):
        self.accepted_collection_list = ["collect"] + sorted(self.collecting_list)
        if self.ros_node and self.ros_node.task_waypoints_pub:
            msg = String()
            msg.data = json.dumps(self.accepted_collection_list)
            self.ros_node.task_waypoints_pub.publish(msg)
        message = QMessageBox(dialog)
        message.setWindowTitle("수거 시작")
        message.setText(", ".join(map(str, self.accepted_collection_list)))
        message.setStyleSheet("font-size: 16px;")
        message.show()
        QTimer.singleShot(2000, message.close)
        dialog.close()

        self.collecting_list.clear()

    def handle_emergency_stop(self):
        QMessageBox.critical(self, "긴급 정지", "로봇이 정지되었습니다.")
        self.robot_status_label.setText("로봇 상태: 긴급 정지")
        self.robot_status_label.setStyleSheet("background-color: red; color: white;")


    def add_order_to_center(self, data):
        try:
            Table_id = data.get("Table_id")
            order_box = QGroupBox(f"테이블 번호: {Table_id}")
            order_box.setFixedSize(350, 650)

            order_layout = QVBoxLayout()
            if any(order.get("Item") == "직원 호출" for order in data.get("order", [])):
                order_box.setStyleSheet("background-color: yellow; padding: 10px; border-radius: 10px;")
                label = QLabel("직원 호출")
                label.setAlignment(Qt.AlignCenter)
                label.setStyleSheet("font-size: 24px; font-weight: bold;")
                order_layout.addWidget(label)

                table_label = QLabel(f"테이블 번호: {Table_id}")
                table_label.setAlignment(Qt.AlignCenter)
                table_label.setStyleSheet("font-size: 24px; font-weight: bold;")
                order_layout.addWidget(table_label)

                accept_button = QPushButton("접수")
                accept_button.setStyleSheet("border: 2px solid black; padding: 5px;")
                accept_button.clicked.connect(lambda: self.remove_order(order_box))
                order_layout.addWidget(accept_button)
            else:
                order_box.setStyleSheet("border: 2px solid #0078d7; padding: 10px; border-radius: 10px;")
                for order in data.get("order", []):
                    Item = order.get("Item")
                    Quantity = order.get("Quantity")
                    Price = order.get("Price")
                    item_label = QLabel(f"메뉴명: {Item} | 수량: {Quantity} | 가격: {Price}원")
                    item_label.setStyleSheet("font-size: 14px;")
                    order_layout.addWidget(item_label)

                total_amount = Price
                amount_label = QLabel(f"총 금액: {total_amount}원")
                amount_label.setStyleSheet("font-size: 14px;")
                order_layout.addWidget(amount_label)

                button_layout = QHBoxLayout()
                accept_button = QPushButton("접수")
                accept_button.clicked.connect(lambda: self.accept_order(order_box, data))
                reject_button = QPushButton("거부")
                reject_button.clicked.connect(lambda: self.reject_order(order_box, data))
                serving_button = QPushButton("서빙 가능")
                serving_button.setEnabled(False)
                serving_button.clicked.connect(lambda: self.add_to_delivering_list(Table_id, order_box))

                button_layout.addWidget(accept_button)
                button_layout.addWidget(reject_button)
                button_layout.addWidget(serving_button)
                order_layout.addLayout(button_layout)
                order_box.serving_button = serving_button

            order_box.setLayout(order_layout)
            self.scroll_layout.addWidget(order_box)
        except Exception as e:
            print(f"Error adding order: {e}")

    def accept_order(self, order_box, data):
        try:
            Table_id = data["Table_id"]
            order_box.setStyleSheet("background-color: lightgreen;")
            QMessageBox.information(self, "접수 완료", "주문이 접수되었습니다.")
            for order in data["order"]:
                Item = order["Item"]
                Quantity = order["Quantity"]
                Price = order["Price"]
                Amount = Price
                self.ros_node.save_order_info(Table_id, Item, Quantity, Price, Amount, "Accepted")
            order_box.serving_button.setEnabled(True)
            self.accepted_orders.add(Table_id)
            if self.start_button:
                self.start_button.setEnabled(True)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"주문 접수 중 문제가 발생했습니다: {e}")

    def reject_order(self, order_box, data):
        try:
            Table_id = order_box.title().split()[-1]
            order_box.setStyleSheet("background-color: pink;")
            QTimer.singleShot(1000, lambda: self.remove_order_safe(order_box))
            QMessageBox.information(self, "거부 완료", "주문이 거부되었습니다.")
            for order in data.get("order", []):
                Item = order["Item"]
                Quantity = order["Quantity"]
                Price = order["Price"]
                Amount = Price
                self.ros_node.save_order_info(Table_id, Item, Quantity, Price, Amount, "Rejected")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"주문 거부 중 문제가 발생했습니다: {e}")

    def add_to_delivering_list(self, Table_id, order_box):
        try:
            self.delivering_list.add(Table_id)
            order_box.setStyleSheet("background-color: lightblue;")
            popup = QMessageBox(self)
            popup.setWindowTitle("서빙 가능")
            popup.setText(f"테이블 {Table_id} 추가: {sorted(self.delivering_list)}")
            popup.setIcon(QMessageBox.Information)
            popup.show()
            QTimer.singleShot(1000, lambda: (popup.close(), self.remove_order(order_box)))
            self.publish_serving_request(Table_id)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"서빙 가능 목록에 추가 중 문제가 발생했습니다: {e}")

    def publish_serving_request(self, Table_id):
        if self.ros_node and self.ros_node.task_waypoints_pub:
            serving_list = ["serve", Table_id]
            msg = String()
            msg.data = json.dumps(serving_list)
            self.ros_node.task_waypoints_pub.publish(msg)

    def remove_order_safe(self, order_box):
        if order_box is not None:
            self.remove_order(order_box)

    def remove_order(self, order_box):
        self.scroll_layout.removeWidget(order_box)
        order_box.deleteLater()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    ros_worker = ROSNodeWorker()
    window = GUI(ros_worker.node)
    ros_worker.table_order_trigger.connect(window.add_order_to_center)
    ros_worker.robot_status_trigger.connect(window.update_robot_status)
    ros_worker.delivered_list_trigger.connect(lambda table_id: window.on_serving_completed(table_id))
    ros_worker.start()

    def cleanup():
        ros_worker.stop()
        ros_worker.wait()

    app.aboutToQuit.connect(cleanup)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error occurred: {e}")
        rclpy.shutdown()
        sys.exit(1)
