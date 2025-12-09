#include "rclcpp/rclcpp.hpp"                  // ROS 2 C++ 클라이언트 라이브러리 (핵심)
#include "sensor_msgs/msg/image.hpp"          // ROS 2 이미지 메시지 타입 정의
#include "cv_bridge/cv_bridge.h"              // ROS 이미지 메시지 <-> OpenCV Mat 간 변환 라이브러리
#include <opencv2/opencv.hpp>                 // OpenCV 컴퓨터 비전 라이브러리 (영상 처리 핵심)
#include <chrono>                             // 시간 관련 함수 (처리 시간 측정용)
#include <iostream>                           // 표준 입출력 (std::cout, std::endl 사용을 위해 포함)

using std::placeholders::_1;                  // 콜백 함수 바인딩을 위한 플레이스홀더
using namespace cv;                           // OpenCV 네임스페이스 사용
using namespace std;                          // 표준 네임스페이스 사용

class LineDetector : public rclcpp::Node
{
public:
  LineDetector()
  : Node("line_detector") // **노드 이름 정의**: 'line_detector'
  {
    // 'video/image_raw' 토픽을 구독하는 구독자 생성
    // QOS(Quality of Service)는 10으로 설정
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video/image_raw", 10, std::bind(&LineDetector::topic_callback, this, _1));
    
    // **라인 추적을 위한 초기 위치 설정** (State variable)
    // 영상의 폭은 640px이므로, 초기 라인 중심 X좌표는 중앙인 320으로 가정
    prev_line_x_ = 320; 
  }

private:
  // **구독 콜백 함수**: 새 이미지 메시지가 도착할 때마다 호출됨
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 1. 시간 측정 시작 (노드 처리 성능 분석용)
    auto start_time = std::chrono::steady_clock::now();

    // ROS2 이미지 메시지를 OpenCV Mat 형식(BGR 3채널)으로 변환
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
    // 2. 전처리 (Preprocessing) 단계
    
    // 2-1. **ROI (Region of Interest) 선정**
    // 로봇과 가까운 영역인 **영상 하단 1/4 영역**만 관심 영역으로 지정
    int roi_y = 270; // 시작 Y좌표 (360px 중 270부터 시작)
    int roi_h = 90;  // 높이 (270 + 90 = 360, 하단 끝까지)
    // ROI 바운딩 박스 정의 (x=0, y=270, width=640, height=90)
    cv::Rect roi_rect(0, roi_y, 640, roi_h);
    cv::Mat roi = img(roi_rect); // 원본 이미지에서 ROI 영역만 잘라내어 roi 변수에 저장

    // 2-2. **그레이스케일 변환**
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY); // 3채널 BGR -> 1채널 Gray

    // 2-3. **밝기 보정 (Illumination compensation)**
    // 조명 변화에 강건성을 확보하기 위해 평균 밝기를 일정 값(100)으로 맞춤
    cv::Scalar mean_val = cv::mean(gray); // 현재 ROI의 평균 밝기 계산
    int brightness_diff = 100 - (int)mean_val[0]; // 목표 밝기(100) - 현재 평균 밝기
    gray = gray + brightness_diff; // 영상 전체 픽셀 값에 밝기 차이만큼 더해 보정

    // 2-4. **이진화 (Thresholding)**
    cv::Mat binary;
    // 임계값 170 적용: 픽셀 값이 170보다 밝으면 흰색(255), 어두우면 검은색(0)으로 변환
    cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);
    
    // 2-5. **모폴로지(열림) 연산을 통한 작은 노이즈 제거**
    // '열림(MORPH_OPEN)' = 침식 후 팽창 (Erosion followed by Dilation)
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 3x3 사각형 커널 정의
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel); // 작은 잡티나 고립된 픽셀 제거

    // 이진화된 흑백 영상(binary)에 검출 결과를 시각화하기 위해 BGR(3채널)로 변환
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // 3. 라인 검출 (Connected Components / 레이블링)
    // 연결된 픽셀 덩어리(객체)를 찾고 통계 정보(면적, 중심 등)를 추출
    cv::Mat labels, stats, centroids;
    // num_labels: 찾은 객체 수 (0번은 배경)
    int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids); 

    int best_line_idx = -1;           // 최종 라인의 레이블 인덱스
    double min_dist = 99999.0;       // 이전 라인 위치(prev_line_x_)와의 최소 거리
    
    // 배경(i=0)을 제외한 모든 후보 객체(i=1부터)를 순회하며 트래킹
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);       // 객체의 면적
        int cx = (int)centroids.at<double>(i, 0);           // 객체의 무게 중심 X좌표 (ROI 기준 0~640)
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);        // 바운딩 박스 시작 X
        int y = stats.at<int>(i, cv::CC_STAT_TOP);         // 바운딩 박스 시작 Y
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);       // 바운딩 박스 너비
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);      // 바운딩 박스 높이

        // 3-1. 노이즈 제거 (면적 필터링)
        // 면적이 너무 작거나 (100 미만) 너무 크면 (5000 초과) 노이즈로 간주하고 무시
        if (area < 100 || area > 5000) continue; 

        // **라인 후보 시각화** (파란색 박스)
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255, 0, 0), 1);
        // **라인 후보 중심점 시각화** (파란색 점)
        cv::circle(binary_color, cv::Point(cx, y + h/2), 3, cv::Scalar(255, 0, 0), -1);

        // 3-2. **트래킹**: 이전 라인 위치(prev_line_x_)와 현재 후보 중심(cx)의 거리 계산
        double dist = std::abs(cx - prev_line_x_);
        // 가장 가까운 객체를 최종 라인으로 임시 선택
        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    int error = 0; // 초기 에러 값은 0
    
    // 4. 최종 라인 결정 및 에러 계산
    if (best_line_idx != -1) {
        // **라인을 찾은 경우** (가장 가까운 객체가 발견됨)
        int cx = (int)centroids.at<double>(best_line_idx, 0); // 최종 라인 중심 X좌표
        int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // **최종 라인 시각화** (빨간색 박스와 점)
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2);
        cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0, 0, 255), -1); 

        // **에러 계산**: 로봇의 목표 중심(320) - 라인 중심(cx)
        // (+) 에러: 라인이 왼쪽에 있음 (로봇은 오른쪽으로 조향해야 함)
        // (-) 에러: 라인이 오른쪽에 있음 (로봇은 왼쪽으로 조향해야 함)
        error = 320 - cx; 
        
        // **추적 위치 업데이트**: 다음 주기를 위해 현재 찾은 라인 위치를 저장
        prev_line_x_ = cx; 
    } else {
        // **라인을 찾지 못한 경우**
        // error 값은 0으로 유지되거나 (현재 로직), 직전 위치를 기반으로 계산하여 직진 유지 (Keep Going)
        // prev_line_x_는 이전 값을 유지하여 직전 경로를 기억함
    }

    // 5. 처리 시간 계산 및 출력
    auto end_time = std::chrono::steady_clock::now();
    float processing_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    // 터미널 출력: 현재 에러 값과 처리 시간 (성능 모니터링)
    std::cout << "error: " << error << ", time: " << processing_time << " ms" << std::endl;

    // 화면 출력: 원본 영상 전체(640x360), 이진화된 결과(결과 포함) 분리 출력
    cv::imshow("Original Full Image", img); 
    cv::imshow("Binary Result View", binary_color); 

    cv::waitKey(1); // GUI 이벤트를 처리하고 다음 프레임까지 대기 (OpenCV 필수)
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_; // 이전 주기 라인 중심 X좌표 (라인 트래킹 상태 변수)
};

int main(int argc, char * argv[])
{
  // ROS 2 시스템 초기화 (필수)
  rclcpp::init(argc, argv);
  
  // 노드 실행: LineDetector 객체를 생성하고 spin (콜백 함수를 반복 호출하며 ROS 이벤트 처리)
  rclcpp::spin(std::make_shared<LineDetector>());
  
  // ROS 2 시스템 종료 (필수)
  rclcpp::shutdown();
  return 0;
}
