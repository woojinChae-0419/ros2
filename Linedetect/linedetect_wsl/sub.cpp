#include "rclcpp/rclcpp.hpp"                  // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/image.hpp"          // 이미지 메시지 타입 정의
#include "cv_bridge/cv_bridge.h"              // ROS 이미지와 OpenCV Mat 간 변환 라이브러리
#include <opencv2/opencv.hpp>                 // OpenCV 라이브러리
#include <chrono>                             // 시간 측정을 위한 라이브러리
#include <iostream>                           // 표준 입출력 (cout/endl)

using std::placeholders::_1;
using namespace cv;
using namespace std;

class LineDetector : public rclcpp::Node
{
public:
  LineDetector()
  : Node("line_detector") // 노드 이름 정의
  {
    // 'video/image_raw' 토픽을 구독하는 구독자 생성 (QoS 10)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video/image_raw", 10, std::bind(&LineDetector::topic_callback, this, _1));
    
    // 라인 추적을 위한 초기 위치 설정. 영상의 중심(320)으로 가정
    prev_line_x_ = 320; 
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 1. 시간 측정 시작 (노드 처리 성능 분석용)
    auto start_time = std::chrono::steady_clock::now();

    // ROS2 메시지를 OpenCV Mat 형식(BGR 3채널)으로 변환
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
    // 2. 전처리 (Preprocessing)
    // 2-1. ROI (Region of Interest) 선정
    // 영상 하단 1/4 영역 (y=270부터 90픽셀 높이)만 관심 영역으로 지정
    int roi_y = 270;
    int roi_h = 90;
    cv::Rect roi_rect(0, roi_y, 640, roi_h); // x=0, y=270, width=640, height=90
    cv::Mat roi = img(roi_rect); // 원본 이미지에서 ROI 영역만 잘라냄

    // 2-2. 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // 2-3. 밝기 보정 (조명 변화에 강건성 확보)
    // 현재 ROI의 평균 밝기를 목표 밝기(100)로 보정
    cv::Scalar mean_val = cv::mean(gray);
    int brightness_diff = 100 - (int)mean_val[0]; // 목표 밝기 - 현재 평균 밝기
    gray = gray + brightness_diff; // 영상 전체에 밝기 차이만큼 더해줌

    // 2-4. 이진화 (Thresholding)
    cv::Mat binary;
    // 임계값 170 적용: 170보다 밝은 픽셀은 흰색(255), 어두운 픽셀은 검은색(0)으로 만듦
    cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);
    
    // 2-5. 모폴로지(열림) 연산을 통한 작은 노이즈 제거
    // 3x3 사각형 커널 사용
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // 열림(Open) 연산: 작은 잡티나 튀어나온 부분을 제거
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // 이진화된 흑백 영상에 검출 결과를 표시하기 위해 BGR(3채널)로 변환
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // 3. 라인 검출 (Connected Components / 레이블링)
    // 연결된 구성 요소(픽셀 덩어리)를 찾음
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids); // 0번 레이블은 배경

    int best_line_idx = -1; // 최종 라인의 레이블 인덱스
    double min_dist = 99999.0; // 이전 라인 위치와의 최소 거리
    
    // 배경(i=0)을 제외한 모든 후보 객체(i=1부터) 순회
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA); // 객체의 면적
        int cx = (int)centroids.at<double>(i, 0);       // 객체의 무게 중심 X좌표 (ROI 기준)
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);    // 바운딩 박스 시작 X좌표
        int y = stats.at<int>(i, cv::CC_STAT_TOP);     // 바운딩 박스 시작 Y좌표
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);   // 바운딩 박스 너비
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);  // 바운딩 박스 높이

        // 3-1. 노이즈 제거 (면적 필터링)
        // 너무 작거나 (100 미만) 너무 큰 (5000 초과) 객체는 노이즈로 간주하고 무시
        if (area < 100 || area > 5000) continue; 

        // 유효한 라인 후보는 파란색 박스로 표시 (시각화)
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255, 0, 0), 1);

        // 3-2. 트래킹: 이전 라인 위치와의 거리 계산
        double dist = std::abs(cx - prev_line_x_);
        // 이전 라인 위치와 가장 가까운 객체를 최종 라인으로 선택
        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    int error = 0;
    if (best_line_idx != -1) {
        // 4. 최종 라인 위치 확정 및 에러 계산
        int cx = (int)centroids.at<double>(best_line_idx, 0); // 최종 라인의 중심 X좌표
        int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // 최종 라인은 빨간색 박스와 점으로 표시 (시각화)
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2);
        cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0, 0, 255), -1); 

        // 에러 계산: 로봇 중심(320) - 라인 중심(cx)
        // 이 에러 값은 다음 단계(제어/판단)에서 로봇의 조향 명령을 계산하는 데 사용됨
        error = 320 - cx; 
        
        // 다음 주기를 위해 현재 찾은 라인 위치 저장 (트래킹 목적)
        prev_line_x_ = cx; 
    } else {
        // 라인을 찾지 못한 경우: prev_line_x_는 이전 값을 유지하며 추적을 시도함
    }

    // 5. 처리 시간 계산 및 출력
    auto end_time = std::chrono::steady_clock::now();
    float processing_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    // 터미널 출력 (에러 값과 처리 시간)
    std::cout << "error: " << error << ", time: " << processing_time << " ms" << std::endl;

    // 화면 출력: 원본 영상 및 처리 결과 시각화
    cv::imshow("Original Full Image", img); 
    cv::imshow("Binary Result View", binary_color); 

    cv::waitKey(1); // GUI 이벤트를 처리하고 다음 프레임으로 넘어갈 준비
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_; // 이전 주기 라인 중심 X좌표 (추적용)
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // 노드 실행: LineDetector 객체를 생성하고 spin (콜백 함수 반복 호출)
  rclcpp::spin(std::make_shared<LineDetector>());
  rclcpp::shutdown();
  return 0;
}
