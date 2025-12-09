#include "rclcpp/rclcpp.hpp"        // ROS2 노드 생성, 로그 출력, 콜백 처리 등 핵심 기능
#include "sensor_msgs/msg/image.hpp" // ROS2 이미지 메시지 타입(sensor_msgs::msg::Image)
#include "cv_bridge/cv_bridge.h"     // ROS 이미지 ↔ OpenCV Mat 변환용 라이브러리
#include <opencv2/opencv.hpp>        // OpenCV의 영상 처리 기능 전체 포함
#include <chrono>                    // 처리 시간 계산을 위한 시간 라이브러리
#include <iostream>                  // 표준 출력 cout 사용

using std::placeholders::_1;         // 콜백 바인딩 시 placeholder(_1) 사용
using namespace cv;                 // OpenCV 함수 이름 줄이기
using namespace std;                // std:: 생략

// ================================================================
//  LineDetector5
//  - 카메라/비디오에서 프레임을 받아 하단 ROI에서 라인을 찾고
//  - 라벨링으로 여러 후보를 찾은 후
//  - 이전 프레임 중심과 가장 가까운 영역을 추적(tracking)
//  - 320 - cx 방식으로 steering error 계산
// ================================================================
class LineDetector5 : public rclcpp::Node
{
public:
  LineDetector5()
  : Node("line_detector_5")  // ROS2 노드 이름 설정
  {
    // ------------------------------------------------------------
    // 이미지 토픽 구독 설정
    //  - 토픽명: video/image_raw
    //  - 큐 사이즈: 10
    //  - 콜백 함수: topic_callback()
    // ------------------------------------------------------------
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video/image_raw",        // 퍼블리셔가 발행하는 이미지 토픽
        10,                       // 큐 버퍼
        std::bind(&LineDetector5::topic_callback, this, _1) // 콜백 등록
    );

    // ------------------------------------------------------------
    // 이전 프레임의 라인 중심 x 좌표 초기값
    //  - 320: 640x480 기준 화면 중앙
    //  - 첫 프레임에서 초기 추적 기준이 됨
    // ------------------------------------------------------------
    prev_line_x_ = 320;
  }

private:

  // ================================================================
  // topic_callback()
  //  - ROS 이미지가 들어올 때마다 자동 호출됨
  //  1) OpenCV Mat으로 변환
  //  2) ROI(하단 부분) 추출
  //  3) 전처리: grayscale → 조명 보정 → threshold → morphology
  //  4) connectedComponentsWithStats로 라벨링
  //  5) 이전 중심과 거리 비교하여 가장 가까운 라벨 선택
  //  6) error 계산(320 - cx)
  // ================================================================
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    //-------------------------------
    // 프레임 처리 시간 측정 시작
    //-------------------------------
    auto start_time = std::chrono::steady_clock::now();

    // 이전 중심 좌표와 가장 멀어질 수 있는 허용 거리
    const int MAX_DIST = 100;

    // ------------------------------------------------------------
    // 1) ROS 이미지 → OpenCV BGR Mat 변환
    //    - cv_bridge::toCvCopy: ROS Image를 Mat으로 복사
    //    - "bgr8": 기본 OpenCV 포맷(8bit BGR)
    // ------------------------------------------------------------
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // ------------------------------------------------------------
    // 2) ROI(관심영역) 설정
    //   이유: 영상 전체 처리 → 느림
    //         바닥 라인은 항상 하단 근처에 존재 → ROI로 제한하여 속도↑
    //   - ROI 범위: y = 270 ~ 360 (높이 90)
    // ------------------------------------------------------------
    int roi_y = 270;
    int roi_h = 90;
    cv::Rect roi_rect(0, roi_y, 640, roi_h); // 전체 width=640, ROI height=90
    cv::Mat roi = img(roi_rect);             // ROI 이미지 잘라내기

    // ------------------------------------------------------------
    // 3) 그레이스케일 변환 (RGB → Gray)
    //    - threshold 전 단계에서 필수
    // ------------------------------------------------------------
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // ------------------------------------------------------------
    // 4) 조명 보정
    //    - 밝기가 변하면 threshold 결과가 흔들림
    //    - 현재 ROI 평균을 구해 목표 밝기 100 근처로 맞춤
    // ------------------------------------------------------------
    cv::Scalar mean_val = cv::mean(gray);      // ROI 평균 밝기
    int brightness_diff = 100 - (int)mean_val[0]; // 목표 밝기(100)과의 차이
    gray = gray + brightness_diff;             // 전체 밝기 이동

    // ------------------------------------------------------------
    // 5) Threshold: 밝은 영역을 라인 후보로 변환
    //    - 170 이상의 픽셀 = 255(흰색)
    //    - 그 외는 0(검은색)
    // ------------------------------------------------------------
    cv::Mat binary;
    cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);

    // ------------------------------------------------------------
    // 6) Morphology Opening(열기연산)
    //    - 작은 점/노이즈 제거
    //    - 연속된 라인만 남기기 위함
    // ------------------------------------------------------------
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // (디버깅용) 라벨링 시 박스 칠할 수 있게 컬러로 변환
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // ------------------------------------------------------------
    // 7) Connected Components (라벨링)
    //    - binary 이미지에서 흰색 영역(연속된 덩어리)을 찾아 ID 부여
    //    - stats: 면적, bounding box 정보 포함
    //    - centroids: 무게중심(x, y)
    // ------------------------------------------------------------
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

    int best_line_idx = -1;  // 선택된 후보 라벨 ID
    double min_dist = 99999; // 이전 프레임 중심과의 최소거리 저장

    // ------------------------------------------------------------
    // 8) 라벨링된 모든 후보 탐색 (i = 1부터 → 0은 배경)
    // ------------------------------------------------------------
    for (int i = 1; i < num_labels; i++) {

        int area = stats.at<int>(i, cv::CC_STAT_AREA);           // 후보 면적
        int cx   = centroids.at<double>(i, 0);                   // 무게중심 X
        int x    = stats.at<int>(i, cv::CC_STAT_LEFT);           // bounding box
        int y    = stats.at<int>(i, cv::CC_STAT_TOP);
        int w    = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h    = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        // 면적 필터링: 너무 작은/큰 영역은 라인이 아님
        if (area < 100 || area > 5000) continue;

        // 디버깅용: 후보를 파란 박스로 표시
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255, 0, 0), 1);

        // 이전 중심(prev_line_x_)과의 거리 계산
        double dist = abs(cx - prev_line_x_);

        // 가장 가까운 후보 선택
        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    // ------------------------------------------------------------
    // 9) 중심 후보가 너무 멀면 라인 유실로 판단
    // ------------------------------------------------------------
    int error = 0;

    if (best_line_idx != -1 && min_dist > MAX_DIST) {
        RCLCPP_WARN(this->get_logger(),
                    "Line 5 lost: Closest candidate too far (%.2f)", min_dist);
        best_line_idx = -1; // 유효하지 않다고 처리
    }

    // ------------------------------------------------------------
    // 10) 최종 라인 중심 선택 → steering error 계산
    // ------------------------------------------------------------
    if (best_line_idx != -1) {
        int cx = centroids.at<double>(best_line_idx, 0);       // 무게중심 x
        int x  = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y  = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w  = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h  = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // 선택된 박스를 빨간색으로 표시
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2);

        // 무게중심 표시 (빨간 점)
        cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0, 0, 255), -1);

        // error 계산: 이미지 가운데(320) - cx
        // +면 좌측, -면 우측에 라인이 있다는 뜻
        error = 320 - cx;

        // 다음 프레임 추적을 위해 중심 업데이트
        prev_line_x_ = cx;
    }

    // ------------------------------------------------------------
    // 11) 처리 시간 출력(ms)
    // ------------------------------------------------------------
    auto end_time = std::chrono::steady_clock::now();
    float processing_time =
        std::chrono::duration<float, std::milli>(end_time - start_time).count();

    cout << "error: " << error << ", time: " << processing_time << " ms" << endl;

    // ------------------------------------------------------------
    // 12) 시각적 디버깅 창 출력
    // ------------------------------------------------------------
    cv::imshow("Original Full Image", img);
    cv::imshow("Binary Result View", binary_color);
    cv::waitKey(1); // 1ms 대기 (UI 갱신)
  }

  // 구독자 핸들 + 상태 변수
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_; // 이전 중심 x (tracking용)
};


// ================================================================
// main()
//  - ROS2 노드 초기화 후 spin()
// ================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                       // ROS2 시스템 초기화
  rclcpp::spin(std::make_shared<LineDetector5>()); // 노드 실행
  rclcpp::shutdown();                             // 종료 처리
  return 0;
}
