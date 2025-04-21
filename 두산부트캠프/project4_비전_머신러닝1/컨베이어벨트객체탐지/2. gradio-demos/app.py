import cv2
import gradio as gr
import requests
import numpy as np
from PIL import Image
from requests.auth import HTTPBasicAuth


# 가상의 비전 AI API URL (예: 객체 탐지 API)
VISION_API_URL = "https://suite-endpoint-api-apne2.superb-ai.com/endpoints/bcf354bb-17cf-4435-b8c7-096fe125107f/inference"
TEAM = "kdt2024_1-5"
ACCESS_KEY = "sVuknNs5mQ3PeSkNkMtvx1mmCTJQsqGj5JqhzSb6"


def process_image(image):
    # 이미지를 OpenCV 형식으로 변환
    image = np.array(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # 이미지를 API에 전송할 수 있는 형식으로 변환
    _, img_encoded = cv2.imencode(".jpg", image)
    img_bytes = img_encoded.tobytes()

    # API 호출 및 결과 받기 - 실습1

    URL = VISION_API_URL
    response = requests.post(
    url=VISION_API_URL,
    auth=HTTPBasicAuth(TEAM, ACCESS_KEY), #팀정보, 엑세스키
    headers={"Content-Type": "image/jpeg"},
    data=img_bytes,
    )


    # API 결과를 바탕으로 박스 그리기 - 실습2
    result = response.json()

    color_map = {
        'RASPBERRY PICO': (0, 0, 255),   # 빨간색
        'HOLE': (255, 0, 0),             # 파란색
        'BOOTSEL': (0, 165, 255),        # 주황색
        'OSCILLATOR': (0, 255, 0),       # 초록색
        'USB': (0, 255, 255),            # 노란색
        'CHIPSET': (255, 0, 255)         # 보라색
    }

    thickness = 2  # 박스 선의 두께

    for obj in result["objects"]:
        class_name = obj["class"]
        start_point = (obj["box"][0], obj["box"][1])  # 시작 좌표
        end_point = (obj["box"][2], obj["box"][3])    # 종료 좌표
        
        # 클래스에 해당하는 색상 가져오기
        color = color_map.get(class_name, (255, 255, 255))  # 기본값은 흰색
        
        # 객체를 둘러싼 사각형 그리기
        cv2.rectangle(image, start_point, end_point, color, thickness)
        
        # 클래스 이름을 사각형 위에 표시
        cv2.putText(image, class_name, (start_point[0], start_point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    # BGR 이미지를 RGB로 변환
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return Image.fromarray(image)


# Gradio 인터페이스 설정
iface = gr.Interface(
    fn=process_image,
    inputs=gr.Image(type="pil"),
    outputs="image",
    title="Vision AI Object Detection",
    description="Upload an image to detect objects using Vision AI.",
)

# 인터페이스 실행
iface.launch()
