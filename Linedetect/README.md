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



# VideoPublisher7 설명

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
2. VideoPublisher7 클래스 정의  
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

# Part 2. VideoPublisher7 클래스 선언

```cpp
class VideoPublisher7 : public rclcpp::Node
{
public:
    VideoPublisher7();
private:
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    int frame_count_;
};
```
### 설명
ROS2 노드는 클래스로 정의함

주요 멤버:

- publisher_ → 이미지 퍼블리셔
- timer_ → 주기 콜백 실행
- cap_ → 비디오 파일을 읽는 OpenCV 객체
- frame_count_ → 퍼블리시한 프레임 수

# Part 3. 생성자: 퍼블리셔, 파일 오픈, 타이머 설정

```cpp
VideoPublisher7::VideoPublisher7()
: Node("video_publisher_7"), cap_(0), frame_count_(0)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "video/image_raw", 10);

    const std::string video_path = "/home/linux/simulation/7_lt_ccw_100rpm_in.mp4";
    cap_.open(video_path);
```

### 설명
- 노드 이름: video_publisher_7
- 퍼블리셔 생성:
- 토픽명: video/image_raw
- QoS 큐 사이즈: 10
- VideoCapture 로 영상 파일 오픈

# 비디오 파일 열기 성공 여부 체크
```cpp
if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening video file 7: %s", video_path.c_str());
} else {
    RCLCPP_INFO(this->get_logger(), "Successfully opened video file 7: %s", video_path.c_str());
}
```
# 설명

- 파일이 올바르게 열리지 않으면 에러 출력
- 성공하면 정상적으로 열린 경로 출력

### 타이머 설정 (30 FPS)

```cpp
double fps = 30.0;
auto period = std::chrono::duration<double>(1.0 / fps);

timer_ = this->create_wall_timer(
    period, std::bind(&VideoPublisher7::timer_callback, this));
```

#설명

- 1/30초(=0.033초)마다 timer_callback 실행
- OpenCV 파일 FPS를 읽는 방법으로도 확장 가능\

### Part 4. timer_callback()
```cpp
cv::Mat frame;
cap_ >> frame;
```
(1) 프레임 읽기
VideoCapture 의 operator>> 를 사용하여 다음 프레임 읽음.
(2) 영상 끝(EOF) 처리 — 자동 루프
```cpp
if (frame.empty()) {
    RCLCPP_INFO(this->get_logger(), "Video 7 loop end. Rewinding...");
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap_ >> frame;
    if (frame.empty()) return;
}
```
# 설명

frame.empty() → 파일 끝 도달
다시 처음(프레임 인덱스 0)으로 이동하여 루프 재생
그래도 비면 파일 자체 문제 → 콜백 종료

(3) cv::Mat → sensor_msgs::Image 변환
```cpp
sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
    .toImageMsg();
```
# 설명

cv_bridge 사용
OpenCV 기본 포맷: bgr8
ROS2 Image 메시지로 포장

(4) 퍼블리시
```cpp
publisher_->publish(*msg);
frame_count_++;
```
토픽 /video/image_raw 로 메시지 발행
사용된 프레임 수 카운트 증가

### Part 5. main() 함수

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher7>());
    rclcpp::shutdown();
    return 0;
}
```
# 설명

ROS2 초기화
VideoPublisher7 노드 생성
spin()으로 콜백(타이머 포함) 계속 실행
종료 시 shutdown

# Publisher CMakeLists.txt 설명



## 변경 1 — VideoPublisher5용 실행 파일 추가
```cmake
add_executable(video_publisher_5 src/pub_video5.cpp)

ament_target_dependencies(
  video_publisher_5
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

target_link_libraries(video_publisher_5 ${OpenCV_LIBS})
 ```
## 변경 2 — VideoPublisher7 추가
```cmake
add_executable(video_publisher_7 src/pub_video7.cpp)

ament_target_dependencies(
  video_publisher_7
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

target_link_libraries(video_publisher_7 ${OpenCV_LIBS})
```
## 변경 3 — 설치 규칙에 두 실행 파일 추가
```cmake
install(TARGETS
  video_publisher_5
  video_publisher_7
  DESTINATION lib/${PROJECT_NAME}
)
```
## 요약
새 영상 퍼블리셔 노드 2개 추가됨: video_publisher_5, video_publisher_7
각 노드가 OpenCV, cv_bridge, sensor_msgs 등을 사용하도록 의존성 설정됨
설치 경로에 두 실행파일이 포함되도록 수정됨

# Publisher package.xml 변경된 부분 요약


## 추가된 의존성(영상 퍼블리셔용)

```xml
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv2</depend>
```
나머지는 기본 ROS2 패키지 구성 요소 그대로 유지


###LineDetector5 – ROS2 라인 검출 노드 설명
### 1. 노드 개요
입력: ROS2 Image 메시지 (video/image_raw)
출력: 터미널에 steering error + 처리 시간(ms) 출력, OpenCV 시각화 창
기능:
하단 ROI 라인 검출
연결 영역 라벨링 → 후보 탐색
이전 프레임 중심과 가장 가까운 후보 선택
화면 중앙(320) 기준 error 계산
시각화: 원본 영상 + 이진 처리 결과

### 2. 전체 코드 구조
헤더 포함
LineDetector5 클래스 정의
생성자 (구독자 설정, 이전 중심 초기화)
topic_callback() – 프레임 처리 및 라인 검출
main() – ROS2 초기화 및 spin

### 3.코드 설명 (파트별)
# Part 1. 헤더 선언부
```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

using std::placeholders::_1;
using namespace cv;
using namespace std;
```
설명
ROS2 기능(rclcpp), 이미지 메시지(sensor_msgs/Image)
cv_bridge: ROS Image ↔ OpenCV Mat 변환
OpenCV: 영상 처리
chrono: 처리 시간 측정
std::placeholders: 콜백 바인딩용

#Part 2. LineDetector5 클래스 선언
```cpp
class LineDetector5 : public rclcpp::Node
{
public:
  LineDetector5();
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_;
};
```
설명
ROS2 노드를 클래스 상속 형태로 정의
주요 멤버:
subscription_: 이미지 구독자
prev_line_x_: 이전 프레임 라인 중심 x (tracking 용)

#Part 3. 생성자: 구독자 설정 및 초기화
```cpp
LineDetector5::LineDetector5()
: Node("line_detector_5")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video/image_raw", 10,
        std::bind(&LineDetector5::topic_callback, this, _1)
    );
    prev_line_x_ = 320; // 초기 화면 중앙
}
```
설명
토픽: video/image_raw 구독
큐 사이즈: 10
콜백: topic_callback() 실행
초기 중심 x = 320 (640x480 화면 중앙 기준)

#Part 4. topic_callback() – 프레임 처리
1) ROS Image → OpenCV Mat 변환
```cpp
cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
```
ROS Image를 OpenCV Mat으로 변환
BGR 8bit 인코딩

2) ROI(관심 영역) 추출
```cpp
int roi_y = 270;
int roi_h = 90;
cv::Rect roi_rect(0, roi_y, 640, roi_h);
cv::Mat roi = img(roi_rect);
```
영상 하단 270~360 영역만 처리
속도 최적화 및 하단 라인 검출 집중

3) 전처리
```cpp
cv::Mat gray;
cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

cv::Scalar mean_val = cv::mean(gray);
int brightness_diff = 100 - (int)mean_val[0];
gray = gray + brightness_diff;

cv::Mat binary;
cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);

cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
```
그레이스케일 변환 → 밝기 보정 → threshold → morphology (노이즈 제거)
이진 이미지 생성

4) 라벨링 (Connected Components)
```cpp
cv::Mat labels, stats, centroids;
int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids);
```
흰색 영역(라인 후보) 라벨링
stats: 면적, bounding box 정보
centroids: 무게중심(x, y)

5) 후보 영역 선택
```cpp
int best_line_idx = -1;
double min_dist = 99999;
for (int i = 1; i < num_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    int cx   = centroids.at<double>(i, 0);
    if (area < 100 || area > 5000) continue;
    double dist = abs(cx - prev_line_x_);
    if (dist < min_dist) {
        min_dist = dist;
        best_line_idx = i;
    }
}
```
너무 작거나 큰 영역 제거
이전 프레임 중심과 최소 거리 후보 선택

6) error 계산
```cpp
int error = 0;
if (best_line_idx != -1 && min_dist <= 100) {
    int cx = centroids.at<double>(best_line_idx, 0);
    error = 320 - cx;
    prev_line_x_ = cx;
}
```
화면 중앙(320) 기준 steering error 계산
다음 프레임 추적용 중심 업데이트

7) 디버깅 및 시각화
```cpp
auto end_time = std::chrono::steady_clock::now();
float processing_time =
    std::chrono::duration<float, std::milli>(end_time - start_time).count();

cout << "error: " << error << ", time: " << processing_time << " ms" << endl;

cv::imshow("Original Full Image", img);
cv::imshow("Binary Result View", binary_color);
cv::waitKey(1);
```
처리 시간(ms) 출력
원본/이진 영상 표시

# Part 5. main() 함수
```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetector5>());
  rclcpp::shutdown();
  return 0;
}
```
ROS2 초기화 → 노드 실행 → 종료 처리

# 4. 요약

ROS2 Image 구독 → 하단 ROI 라인 검출 → steering error 계산
연결 영역 라벨링 → 후보 중심과 이전 중심 거리 비교 → 최적 후보 선택
처리 시간 및 화면 출력으로 디버깅 가능
자율주행 테스트 및 시뮬레이션용 라인 트래킹 노드


### LineDetector7 – ROS2 라인 검출 노드 설명
###1. 노드 개요
입력: ROS2 Image 메시지 (video/image_raw)
출력: 터미널에 steering error + 처리 시간(ms) 출력, OpenCV 시각화 창
기능:
하단 ROI 라인 검출
연결 영역 라벨링 → 후보 탐색
이전 프레임 중심과 가장 가까운 후보 선택
화면 중앙(320) 기준 error 계산
시각화: 원본 영상 + 이진 처리 결과

### 2. 전체 코드 구조
1)헤더 포함
2)LineDetector7 클래스 정의
3)생성자 (구독자 설정, 이전 중심 초기화)
4)topic_callback() – 프레임 처리 및 라인 검출
5)main() – ROS2 초기화 및 spin

### 3. 코드 설명 (파트별)
# Part 1. 헤더 선언부
```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

using std::placeholders::_1;
using namespace cv;
using namespace std;
```
설명

ROS2 기능(rclcpp), 이미지 메시지(sensor_msgs/Image)

cv_bridge: ROS Image ↔ OpenCV Mat 변환

OpenCV: 영상 처리 기능 전체 포함

chrono: 처리 시간 측정

std::placeholders: 콜백 바인딩용

# Part 2. LineDetector7 클래스 선언
```cpp
class LineDetector7 : public rclcpp::Node
{
public:
  LineDetector7();
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_;
};
```
설명

ROS2 노드를 클래스 상속 형태로 정의

주요 멤버:

subscription_: 이미지 구독자

prev_line_x_: 이전 프레임 라인 중심 x (tracking 용)

# Part 3. 생성자: 구독자 설정 및 초기화
```cpp
LineDetector7::LineDetector7()
: Node("line_detector_7")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video/image_raw", 10,
        std::bind(&LineDetector7::topic_callback, this, _1)
    );
    
    prev_line_x_ = 320; // 초기 화면 중앙
}
```
설명

토픽: video/image_raw 구독

큐 사이즈: 10

콜백: topic_callback() 실행

초기 중심 x = 320 (640x480 화면 중앙 기준)

# Part 4. topic_callback() – 프레임 처리
1) ROS Image → OpenCV Mat 변환
```cpp
cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
```
ROS Image를 OpenCV Mat으로 변환

BGR 8bit 인코딩

2) ROI(관심 영역) 추출
```cpp
int roi_y = 270;
int roi_h = 90;
cv::Rect roi_rect(0, roi_y, 640, roi_h);
cv::Mat roi = img(roi_rect);
```
영상 하단 270~360 영역만 처리

연산 최적화 및 노이즈 감소

3) 전처리: 그레이스케일 + 밝기 보정
```cpp
cv::Mat gray;
cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

cv::Scalar mean_val = cv::mean(gray);
int brightness_diff = 100 - (int)mean_val[0];
gray = gray + brightness_diff;
```
RGB → Gray 변환

평균 밝기 기반 보정 → 조도 변화에 강함

4) 이진화 + Morphology
```cpp
cv::Mat binary;
cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);

cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

cv::Mat binary_color;
cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);
```
밝은 영역(라인)을 흰색으로 분리

작은 점 제거(Open 연산)

시각화를 위해 컬러 변환

(5)
```cpp
cv::Mat labels, stats, centroids;
int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids);
```
각 영역에 라벨 부여

stats: 면적, bounding box 정보

centroids: 중심 좌표(x, y)
6) 후보 영역 선택
```cpp
int best_line_idx = -1;
double min_dist = 99999.0;

for (int i = 1; i < num_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    int cx = (int)centroids.at<double>(i, 0);
    if (area < 100 || area > 5000) continue;

    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255,0,0),1);
    cv::circle(binary_color, cv::Point(cx, y + h/2), 3, cv::Scalar(255,0,0),-1);

    double dist = abs(cx - prev_line_x_);
    if (dist < min_dist) {
        min_dist = dist;
        best_line_idx = i;
    }
}
```
후보 영역 필터링: 면적 100~5000

이전 중심과 거리 비교 → 가장 가까운 후보 선택

디버깅용 파란 박스 + 점 표시

7) 최종 라인 선택 + error 계산
```cpp
int error = 0;

if (best_line_idx != -1 && min_dist < 80.0) {
    int cx = (int)centroids.at<double>(best_line_idx, 0);
    int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
    int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
    int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

    cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0,0,255),2);
    cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0,0,255),-1);

    error = 320 - cx;
    prev_line_x_ = cx;
} else {
    error = 320 - prev_line_x_;
}
```
조건:

후보 존재

이전 중심과 거리 < 80px

선택 영역 빨간색 표시

화면 중심과 차이 → steering error

8) 처리 시간 계산
```cpp
auto end_time = std::chrono::steady_clock::now();
float processing_time = std::chrono::duration<float,std::milli>(end_time - start_time).count();
cout << "error: " << error << ", time: " << processing_time << " ms" << endl;
```
프레임당 처리 시간 측정

error와 함께 출력

9) 화면 출력
```cpp
cv::imshow("Original Full Image", img);
cv::imshow("Binary Result View", binary_color);
cv::waitKey(1);
```
원본 영상 + 이진화 결과 출력

실시간 디버깅용

# Part 5. main() 함수
```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetector7>());
  rclcpp::shutdown();
  return 0;
}
```
ROS2 초기화 → 노드 실행 → 종료 처리

### 4. 요약

LineDetector7는 LineDetector5와 구조 유사하지만

후보 거리 허용값(min_dist < 80px)으로 안정적 추적

시각적 디버깅용 박스/점 추가

이전 중심 없을 시 error 유지

하단 ROI만 처리 → 연산 최적화

밝기 보정, threshold, 라벨링 → 안정적 라인 검출

화면 중심 기준 error 출력 → 자율주행 제어 가능
