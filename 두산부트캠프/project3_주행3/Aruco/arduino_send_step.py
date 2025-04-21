import serial
import time

# 시리얼 포트와 통신 설정
arduino_port = '/dev/ttyACM0'  # Windows에서는 COM 포트 번호, Mac/Linux에서는 '/dev/ttyUSB0' 또는 '/dev/ttyACM0'
baud_rate = 115200       # Arduino와 동일한 baud rate
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

def send_data(data):
    if not ser.is_open:
        ser.open()
    ser.write(data.encode())  # 데이터를 바이트로 변환하여 전송
    print(f"Sent: {data}")

try:
    while True:
        # 사용자로부터 입력받아 Arduino로 전송
        input_value = input("Enter a number to send to Arduino (or 'exit' to quit): ")
        if input_value.lower() == 'exit':  # 'exit' 입력 시 종료
            break
        if input_value.isdigit():  # 입력이 숫자인지 확인
            send_data(input_value + '\n')  # 숫자와 '\n' 전송 (아두이노가 문자열 끝을 인식할 수 있도록)
        else:
            print("Please enter a valid number.")



except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    ser.close()
    print("Serial connection closed.")
