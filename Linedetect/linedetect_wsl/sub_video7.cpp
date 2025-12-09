#include "rclcpp/rclcpp.hpp"                  // ROS 2 C++ 클라이언트 라이브러리 (핵심)
#include "sensor_msgs/msg/image.hpp"          // ROS 2 이미지 메시지 타입 정의
#include "cv_bridge/cv_bridge.h"              // ROS 이미지 메시지 <-> OpenCV Mat 간 변환 라이브러리
#include <opencv2/opencv.hpp>                 // OpenCV 컴퓨터 비전 라이브러리 (영상 처리 핵심)
#include <chrono>                             // 시간 관련 함수 (처리 시간 측정용)
#include <iostream>                           // 표준 입출력 (std::cout, std::endl 사용)

using std::placeholders::_1;                  // 콜백 함수 바인딩을 위한 플레이스홀더
using namespace cv;                           // OpenCV 네임스페이스 사용
using namespace std;                          // 표준 네임스페이스 사용

class LineDetector7 : public rclcpp::Node
{
public:
  LineDetector7()
  : Node("line_detector_7") // **노드 이름 정의**: 'line_detector_7'
  {
    // 'video/image_raw' 토픽을 구독하는 구독자 생성 (QoS 10)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video/image_raw", 10, std::bind(&LineDetector7::topic_callback, this, _1));
    
    // **라인 추적을 위한 초기 위치 설정**
    // 영상 중앙(320)으로 시작 위치 가정
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
    // 영상 하단 1/4 영역 (y=270부터 90픽셀 높이)만 관심 영역으로 지정
    int roi_y = 270; 
    int roi_h = 90;  
    cv::Rect roi_rect(0, roi_y, 640, roi_h); 
    cv::Mat roi = img(roi_rect); // 원본 이미지에서 ROI 영역만 잘라냄

    // 2-2. **그레이스케일 변환**
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // 2-3. **밝기 보정 (Illumination compensation)**
    // 현재 ROI의 평균 밝기를 목표 밝기(100)로 보정하여 조명 변화에 대응
    cv::Scalar mean_val = cv::mean(gray); 
    int brightness_diff = 100 - (int)mean_val[0]; // 목표 밝기 - 현재 평균 밝기
    gray = gray + brightness_diff; // 영상 전체에 밝기 차이만큼 더해 보정

    // 2-4. **이진화 (Thresholding)**
    cv::Mat binary;
    // 임계값 170 적용: 170보다 밝으면 흰색(255), 어두우면 검은색(0)으로 변환
    cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);
    
    // 2-5. **모폴로지(열림) 연산을 통한 작은 노이즈 제거**
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); 
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel); // 열림 연산으로 잡티 제거

    // 이진화된 흑백 영상에 검출 결과를 시각화하기 위해 BGR(3채널)로 변환
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // 3. 라인 검출 (Connected Components / 레이블링)
    // 연결된 픽셀 덩어리(객체)를 찾고 통계 정보(면적, 중심 등)를 추출
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids); // 0번은 배경

    int best_line_idx = -1;           // 최종 라인의 레이블 인덱스
    double min_dist = 99999.0;       // 이전 라인 위치와의 최소 거리
    
    // 배경(i=0)을 제외한 모든 후보 객체(i=1부터)를 순회
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);       // 객체의 면적
        int cx = (int)centroids.at<double>(i, 0);           // 객체의 무게 중심 X좌표
        
        // 3-1. 노이즈 제거 (면적 필터링)
        // 너무 작거나 (100 미만) 너무 큰 (5000 초과) 객체는 무시
        if (area < 100 || area > 5000) continue; 

        int x = stats.at<int>(i, cv::CC_STAT_LEFT);        
        int y = stats.at<int>(i, cv::CC_STAT_TOP);         
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);       
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);      

        // **라인 후보 시각화**: 파란색 박스와 중심점 그리기
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255, 0, 0), 1); // 파란색 박스
        cv::circle(binary_color, cv::Point(cx, y + h/2), 3, cv::Scalar(255, 0, 0), -1); // 파란색 점

        // 3-2. **트래킹**: 이전 라인 위치(prev_line_x_)와 현재 후보 중심(cx)의 거리 계산
        double dist = std::abs(cx - prev_line_x_);
        // 이전 라인 위치와 가장 가까운 객체를 최종 라인으로 임시 선택
        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    int error = 0;
    
    // 4. 최종 라인 결정 및 에러 계산 (강화된 트래킹 로직)
    // **80픽셀 이내일 경우만** 이전 라인과 동일한 라인으로 인정 (바깥쪽 선/노이즈 무시 필터)
    if (best_line_idx != -1 && min_dist < 80.0) {
        // **라인 추적에 성공한 경우**
        int cx = (int)centroids.at<double>(best_line_idx, 0); // 최종 라인 중심 X좌표
        int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // **최종 라인 시각화**: 빨간색 박스와 점으로 표시 (추적 성공 확인)
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2);
        cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0, 0, 255), -1); 

        // **에러 계산**
        error = 320 - cx; // 로봇 목표 중심(320) - 라인 중심(cx)
        prev_line_x_ = cx; // **추적 위치 업데이트**: 다음 주기를 위해 현재 위치 저장
    } else {
        // **라인을 찾지 못했거나 (유실), 너무 멀리 떨어진 경우 (바깥쪽 선)**
        
        // 4-2. **에러 계산 (Keep Going)**
        // 에러는 **이전 라인 위치**를 기준으로 계산하여, 라인이 유실되어도 **직진/직전 방향을 유지**하도록 지시
        error = 320 - prev_line_x_;
        
        // **추적 위치는 업데이트하지 않음**: prev_line_x_는 이전 값을 유지한 채 추적을 지속 시도
    }

    // 5. 처리 시간 계산 및 출력
    auto end_time = std::chrono::steady_clock::now();
    float processing_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    // 터미널 출력: 현재 에러 값과 처리 시간
    std::cout << "error: " << error << ", time: " << processing_time << " ms" << std::endl;

    // 화면 출력: 원본 영상, 이진화된 결과(시각화 포함) 분리 출력
    cv::imshow("Original Full Image", img); 
    cv::imshow("Binary Result View", binary_color); 

    cv::waitKey(1); // GUI 이벤트를 처리하고 다음 프레임으로 넘어갈 준비 (OpenCV 필수)
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_; // 이전 주기 라인 중심 X좌표 (라인 트래킹 상태 변수)
};

int main(int argc, char * argv[])
{
  // ROS 2 시스템 초기화 (필수)
  rclcpp::init(argc, argv);
  
  // 노드 실행: LineDetector7 객체를 생성하고 spin (ROS 이벤트 반복 처리)
  rclcpp::spin(std::make_shared<LineDetector7>());
  
  // ROS 2 시스템 종료 (필수)
  rclcpp::shutdown();
  return 0;
}
