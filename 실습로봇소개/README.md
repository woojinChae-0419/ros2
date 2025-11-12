실습과제 1.
(1)
<img width="735" height="529" alt="화면 캡처 2025-11-12 150656" src="https://github.com/user-attachments/assets/659c7619-3853-47b1-968f-023ec102db1c" />
(2)
<img width="908" height="792" alt="image" src="https://github.com/user-attachments/assets/9e60943c-4435-4142-90a8-0e293b9c766d" />
(3)
<img width="643" height="386" alt="image" src="https://github.com/user-attachments/assets/52c75fc7-aa7e-4964-a389-d04823098780" />
(4)
<img width="460" height="313" alt="image" src="https://github.com/user-attachments/assets/9262f96f-7dda-4235-b9d8-d249d0afa471" />
(5)
<img width="941" height="692" alt="image" src="https://github.com/user-attachments/assets/844bf76f-c9a6-43cc-a6cb-4b2c95b09450" />


실습과제2.

etson Nano

NVIDIA의 소형 AI 컴퓨터.

GPU(128코어) + ARM CPU 탑재.

카메라, 로봇, 인공지능 실습용 보드.

IMX219 카메라

Sony 8메가픽셀 이미지 센서.

해상도 3280×2464.

MIPI CSI-2 인터페이스 사용.

카메라 리눅스 장치파일

보통 /dev/video0 으로 표시됨.

CSI (Camera Serial Interface)

카메라와 보드를 연결하는 고속 직렬 영상 인터페이스.

MIPI CSI-2 방식이 가장 일반적.

GStreamer

오디오/비디오를 처리하는 오픈소스 멀티미디어 프레임워크.

gst-launch-1.0 명령으로 카메라 영상 등을 다룰 수 있음.

DYNAMIXEL

로보티즈(ROBOTIS)에서 만든 스마트 서보모터.

위치·속도·토크 제어 가능, ID로 여러 개 연결 가능.

U2D2

PC(USB) ↔ DYNAMIXEL(통신버스) 변환기.

즉, USB로 모터 제어할 수 있게 해주는 장치.

U2D2 리눅스 장치파일 이름

보통 /dev/ttyUSB0 으로 인식됨.
