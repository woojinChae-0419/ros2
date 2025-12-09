5번 영상
https://youtu.be/OCvtC1geFB0
7번 영상
https://www.youtube.com/watch?v=u0kzce874GU

# VideoPublisher5 설명

이 노드는 OpenCV로 mp4 파일을 읽어 ROS2 Image 메시지(sensor_msgs/msg/Image)로 퍼블리시하는 기능을 제공합니다.  
카메라 없이 영상 파일을 입력 소스로 사용하는 실험 및 시뮬레이션에서 유용합니다.

---

# 1. 노드 개요

- 입력: mp4 영상 파일  
- 출력: ROS2 Image 메시지(`video/image_raw`)  
- FPS: 30 FPS 기준 타이머 기반 퍼블리시  
- 기능: 영상 끝에 도달하면 자동 재생(loop)  

---

# 2. 전체 코드 구조

코드는 다음과 같은 파트로 구성됩니다.

1. 헤더 포함  
2. VideoPublisher5 클래스 정의  
3. 생성자(퍼블리셔, 비디오 오픈, 타이머 설정)  
4. timer_callback() - 프레임 읽기 및 Image 메시지 변환  
5. main() - 노드 실행  

아래에서 각 파트를 나누어 설명합니다.

---

# 3. 코드 설명 (파트별)

---

## Part 1. 헤더 선언부

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;
```

### 설명
- ROS2 기능(rclcpp), 이미지 메시지(sensor_msgs/Image)
- cv_bridge: OpenCV Mat → ROS Image 변환
- OpenCV: 영상 처리
- chrono: 타이머 주기 설정(예: 33ms = 30fps)

---

## Part 2. VideoPublisher5 클래스 선언

```cpp
class VideoPublisher5 : public rclcpp::Node
{
public:
    VideoPublisher5();
private:
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    int frame_count_;
};
```

### 설명
- ROS2 노드는 클래스로 정의함  
- 주요 멤버:
  - `publisher_` → 이미지 퍼블리셔  
  - `timer_` → 주기 콜백 실행  
  - `cap_` → 비디오 파일을 읽는 OpenCV 객체  
  - `frame_count_` → 퍼블리시한 프레임 수  

---

## Part 3. 생성자: 퍼블리셔, 파일 오픈, 타이머 설정

```cpp
VideoPublisher5::VideoPublisher5()
: Node("video_publisher_5"), cap_(0), frame_count_(0)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "video/image_raw", 10);

    const std::string video_path = "/home/linux/simulation/5_lt_cw_100rpm_out.mp4";
    cap_.open(video_path);
```

### 설명
- 노드 이름: `video_publisher_5`  
- 퍼블리셔 생성:
  - 토픽명: `video/image_raw`
  - QoS 큐 사이즈: `10`  
- VideoCapture 로 영상 파일 오픈

---

### 비디오 파일 열기 성공 여부 체크

```cpp
if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening video file 5: %s", video_path.c_str());
} else {
    RCLCPP_INFO(this->get_logger(), "Successfully opened video file 5: %s", video_path.c_str());
}
```

### 설명
- 파일이 올바르게 열리지 않으면 에러 출력
- 성공하면 정상적으로 열린 경로 출력  

---

### 타이머 설정 (30 FPS)

```cpp
double fps = 30.0;
auto period = std::chrono::duration<double>(1.0 / fps);

timer_ = this->create_wall_timer(
    period, std::bind(&VideoPublisher5::timer_callback, this));
```

### 설명
- 1/30초(=0.033초)마다 timer_callback 실행  
- OpenCV 파일 FPS를 읽는 방법으로도 확장 가능  

---

## Part 4. timer_callback()  
프레임을 읽어서 ROS 메시지로 변환 후 퍼블리시한다.

```cpp
cv::Mat frame;
cap_ >> frame;
```

### (1) 프레임 읽기  
VideoCapture 의 operator>> 를 사용하여 다음 프레임 읽음.

---

### (2) 영상 끝(EOF) 처리 — 자동 루프

```cpp
if (frame.empty()) {
    RCLCPP_INFO(this->get_logger(), "Video 5 loop end. Rewinding...");
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap_ >> frame;
    if (frame.empty()) return;
}
```

### 설명
- frame.empty() → 파일 끝 도달
- 다시 처음(프레임 인덱스 0)으로 이동하여 루프 재생  
- 그래도 비면 파일 자체 문제 → 콜백 종료  

---

### (3) cv::Mat → sensor_msgs::Image 변환

```cpp
sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
    .toImageMsg();
```

### 설명
- cv_bridge 사용  
- OpenCV 기본 포맷: `bgr8`
- ROS2 Image 메시지로 포장  

---

### (4) 퍼블리시

```cpp
publisher_->publish(*msg);
frame_count_++;
```

- 토픽 `/video/image_raw` 로 메시지 발행  
- 사용된 프레임 수 카운트 증가  

---

## Part 5. main() 함수

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher5>());
    rclcpp::shutdown();
    return 0;
}
```

### 설명
- ROS2 초기화  
- VideoPublisher5 노드 생성  
- spin()으로 콜백(타이머 포함) 계속 실행  
- 종료 시 shutdown  

---

# 4. 실행 방법

### 빌드
```bash
colcon build --packages-select linedetect_nano
source install/setup.bash
```

### 실행
```bash
ros2 run linedetect_nano video_publisher_5
```

---

# 5. 퍼블리시되는 토픽

| Topic | Type | 설명 |
|------|------|------|
| `/video/image_raw` | `sensor_msgs/msg/Image` | 프레임 기반 영상 메시지 |

---

# 6. 요약

- OpenCV 영상 파일 → ROS2 Image 퍼블리시  
- 타이머 기반 30 FPS  
- 영상 끝나면 자동 재생  
- cv_bridge 기반 간단하고 확장 가능한 구조  

VideoPublisher7 Node 설명

2. 코드 구조 상세 설명
2-1. 헤더 포함부

rclcpp
ROS2 노드 생성, 로그 출력, 스핀 등을 제공.

sensor_msgs/msg/image.hpp
이미지 메시지 타입 선언.

cv_bridge
cv::Mat과 sensor_msgs::msg::Image 변환.

opencv2/opencv.hpp
비디오 읽기와 영상 처리 기능.

chrono
타이머 주기 설정에 사용.
