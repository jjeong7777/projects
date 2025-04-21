import serial
import time 
# 시리얼 포트와 통신 설정
arduino_port = '/dev/ttyACM0'  # Linux/Mac: '/dev/ttyACM0', Windows: 'COMx'
baud_rate = 115200            # Arduino와 동일한 baud rate
ser = serial.Serial(arduino_port, baud_rate, timeout=1)


# Arduino에 데이터를 보내는 함수
def send_data(data):
    if not ser.is_open:
        ser.open()
    ser.write(data.encode())  # 데이터를 바이트로 변환하여 전송
    print(f"Sent: {data}")

# Arduino의 출력을 읽는 함수
try:
    while True:
        # 사용자로부터 입력받아 Arduino로 전송
        input_value = input("Enter a number to send to Arduino (or 'exit' to quit): ")
        if input_value.lower() == 'exit':  # 'exit' 입력 시 종료
            break
        if input_value.isdigit():  # 입력이 숫자인지 확인
            
            # 입력 및 출력 버퍼 초기화
            ser.reset_input_buffer()  # 이전 입력 데이터를 삭제
            ser.reset_output_buffer()  # 이전 출력 데이터를 삭제
            
            send_data(input_value + '\n')  # 숫자와 '\n' 전송 (아두이노가 문자열 끝을 인식할 수 있도록)
            
            #time.sleep(0.5)  # 아두이노가 응답할 시간을 확보
            # 데이터 읽기
            while ser.read(1):  # 1바이트 읽어 옴 
                try:
                    byte_data = ser.read(1)  # 1바이트 읽기
                    char = byte_data.decode('utf-8') # 바이트 데이터를 ASCII 문자로 변환 # 아두이노 사용시 decode 필요
                    if char == '.':
                        break
                    elif char == '_':
                        print(" ... 작업 중 ...")
                        
                except Exception as e:
                    print(f"Error decoding data: {e}")  
                    
            print("-----------단위 작업 완료------------")
        else:
            print("Please enter a valid number.")
            

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    ser.close()
    print("Serial connection closed.")
