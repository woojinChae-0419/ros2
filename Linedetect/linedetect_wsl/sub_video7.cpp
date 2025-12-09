#include "rclcpp/rclcpp.hpp"                  // ROS2 노드를 생성·관리하고, 로그 출력, 타이머 등을 사용하기 위한 핵심 헤더
#include "sensor_msgs/msg/image.hpp"          // ROS2에서 사용하는 Image 메시지 타입 (sensor_msgs/Image)
#include "cv_bridge/cv_bridge.h"              // ROS Image ↔ OpenCV Mat 변환을 담당하는 브릿지 라이브러리
#include <opencv2/opencv.hpp>                 // OpenCV 기본 기능 (영상 처리, 그레이 변환, 이진화, 컨투어 등)
#include <chrono>                             // 처리 시간 측정용 C++ 표준 라이브러리
#include <iostream>

using std::placeholders::_1;
using namespace cv;
using namespace std;

// =====================================================================
//                   LineDetector7 – 라인 검출 및 error 계산 노드
// =====================================================================
// 이 노드는 ROS2에서 전달되는 영상(topic: video/image_raw)를 받아서
// 1) ROI 설정 (아래쪽 90px만 사용 → 연산 속도 향상)
// 2) 밝기 보정 → 조도 변화에 강하게 만듦
// 3) Threshold → 흰색 라인을 쉽게 분리
// 4) Connected Components (라벨링)으로 라인 후보 영역 추출
// 5) 이전 라인 중심(prev_line_x_)과 가장 가까운 영역만 선택 → 안정적 트래킹
// 6) 이미지 중심(320)과 라인 중심의 차이를 error로 출력
// =====================================================================
class LineDetector7 : public rclcpp::Node
{
public:

  LineDetector7()
  : Node("line_detector_7")   // 노드 이름 (같은 기능 다른 버전 테스트용으로 번호 분리)
  {
    // ------------------------------------------------------------------
    // ROS2 이미지 토픽 구독 설정
    // ------------------------------------------------------------------
    // - 구독 대상: "video/image_raw"
    // - 메시지 타입: sensor_msgs::msg::Image
    // - 큐 크기: 10
    // - 콜백 함수: topic_callback
    // ------------------------------------------------------------------
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video/image_raw",        // 구독할 토픽 이름
      10,                        // 큐 사이즈
      std::bind(&LineDetector7::topic_callback, this, _1) // 콜백 연결
    );
    
    // 이전 프레임에서 찾은 라인 중심 x값 저장.
    // 초기에는 화면 정중앙(320px)으로 설정해서 첫 프레임 선택 기준을 삼음.
    prev_line_x_ = 320;
  }

private:

  // =====================================================================
  //                     이미지 수신 콜백 함수
  // =====================================================================
  // 매 프레임 호출됨.
  // msg: sensor_msgs::msg::Image 형식의 이미지 한 프레임.
  // 내부에서 OpenCV 영상 처리 수행.
  // =====================================================================
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // ------------------------------------------------------------
    // ⏱ 처리 시간 측정 시작 (성능 로그 출력용)
    // ------------------------------------------------------------
    auto start_time = std::chrono::steady_clock::now();

    // ------------------------------------------------------------
    // 1. ROS2 Image 메시지 → OpenCV Mat(BGR8)로 변환
    // ------------------------------------------------------------
    // - ROS에서 받은 이미지는 sensor_msgs/Image 형식
    // - cv_bridge를 통해 OpenCV 형식(Mat)으로 변환해야 영상 처리 가능
    // ------------------------------------------------------------
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
    // =====================================================================
    //                    2. ROI 설정 (이미지 아래쪽만 사용)
    // =====================================================================
    // - 주행 영상에서는 차선이 주로 화면 하단에 위치하므로,
    //   전체 480px 중 마지막 90px만 사용하여 효율 극대화.
    //   → 연산 시간이 크게 감소하고, 노이즈도 줄어듦.
    // - ROI 영역: y = 270 ~ 360
    // =====================================================================
    int roi_y = 270;                 
    int roi_h = 90;

    cv::Rect roi_rect(0, roi_y, 640, roi_h);  // (x, y, width, height)
    cv::Mat roi = img(roi_rect);               // ROI 추출

    // =====================================================================
    //       3. 그레이스케일 변환 + 밝기 보정(조도 변화 보완)
    // =====================================================================
    // (1) RGB → Gray 변환  
    // (2) 평균 밝기를 읽어 들여서 목표 밝기(100)로 강제 이동  
    //     → 조명 어둡거나 밝아도 threshold 민감도 감소  
    // =====================================================================
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // ROI 평균 밝기
    cv::Scalar mean_val = cv::mean(gray);
    // 목표 밝기 100과 현재 밝기 차이 계산
    int brightness_diff = 100 - (int)mean_val[0];
    // 전체 픽셀 밝기 이동 → 전체 ROI 밝기를 일정하게 유지
    gray = gray + brightness_diff;

    // =====================================================================
    //                        4. 이진화 (Threshold)
    // =====================================================================
    // threshold = 170
    // 회색조에서 밝은 부분(라인)을 255(흰색)로 설정하여 형태 분리하기 위함.
    // =====================================================================
    cv::Mat binary;
    cv::threshold(gray, binary, 170, 255, cv::THRESH_BINARY);
    
    // ● 작은 점/노이즈 제거를 위한 열기(Open) 연산 (침식→팽창)
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // 시각화를 위한 컬러 버전 (Binary는 1채널 → 3채널 BGR로 변경)
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // =====================================================================
    //             5. Connected Components(라벨링)
    // =====================================================================
    // 흰색(255) 부분을 각각 하나의 “영역”으로 나누어서
    // 각 라인의 중심점, 면적, bounding box 등 다양한 정보를 얻음.
    // =====================================================================
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
        binary,       // 입력 이진 이미지
        labels,       // 결과 라벨 맵
        stats,        // 각 영역의 bounding box + area 등 정보
        centroids     // 각 영역의 중심점(x, y)
    );

    // 최종 선택된 라인 ID (-1이면 유효한 객체 없음)
    int best_line_idx = -1;
    double min_dist = 99999.0;     // 이전 중심과의 최소 거리

    // =====================================================================
    //        6. 검출된 모든 라인 후보 영역 탐색
    // =====================================================================
    // i = 0 → 배경(background) 이므로 제외
    // 1~num_labels-1 까지가 실제 객체들
    // =====================================================================
    for (int i = 1; i < num_labels; i++) {

        // 각 영역에 대한 면적 정보
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        // 중심점 x
        int cx = (int)centroids.at<double>(i, 0);

        // 너무 작거나 너무 큰 영역 제외 (100px 미만 노이즈, 5000px 초과 큰 물체)
        if (area < 100 || area > 5000) continue;

        // bounding box 정보
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        // ------------------------------------------
        // 모든 후보: 파란색(BGR: 255,0,0)으로 표시
        // ------------------------------------------
        cv::rectangle(binary_color, cv::Rect(x, y, w, h),
                      cv::Scalar(255, 0, 0), 1);
        cv::circle(binary_color, cv::Point(cx, y + h/2),
                   3, cv::Scalar(255, 0, 0), -1);

        // 이전 중심(prev_line_x_)과의 거리 계산 → 가장 가까운 객체 선택
        double dist = abs(cx - prev_line_x_);

        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    // =====================================================================
    //            7. 최종 라인 선택 + error 계산
    // =====================================================================
    // 조건:
    // ① 후보 존재(best_line_idx != -1)
    // ② 이전 중심(prev_line_x_)과의 거리 80px 이하일 때만 인정
    // =====================================================================
    int error = 0;

    if (best_line_idx != -1 && min_dist < 80.0) {

        // 선택된 라인의 중심/박스 정보 가져오기
        int cx = (int)centroids.at<double>(best_line_idx, 0);
        int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // 선택된 라인은 빨간색으로 강조
        cv::rectangle(binary_color, cv::Rect(x, y, w, h),
                      cv::Scalar(0, 0, 255), 2);
        cv::circle(binary_color, cv::Point(cx, y + h/2),
                   4, cv::Scalar(0, 0, 255), -1);

        // 화면 중심(320)과 cx 차이 → steering error
        error = 320 - cx;

        // 현재 중심을 다음 프레임 기준점으로 저장
        prev_line_x_ = cx;

    } else {
        // 라인 못 찾으면 이전 중심 기준으로 error 유지
        error = 320 - prev_line_x_;
    }

    // =====================================================================
    //                        8. 처리 시간 계산
    // =====================================================================
    auto end_time = std::chrono::steady_clock::now();
    float processing_time =
        std::chrono::duration<float, std::milli>(end_time - start_time).count();

    std::cout << "error: " << error << ", time: "
              << processing_time << " ms" << std::endl;

    // =====================================================================
    //                           9. 화면 출력
    // =====================================================================
    cv::imshow("Original Full Image", img); 
    cv::imshow("Binary Result View", binary_color); 

    cv::waitKey(1);
  }

  // ------------------------------------------------------------
  // 멤버 변수: 구독자 핸들 + 이전 라인 중심
  // ------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_;   // 트래킹 안정성 확보용
};

// =====================================================================
//                                main()
// =====================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                        // ROS2 시스템 초기화
  rclcpp::spin(std::make_shared<LineDetector7>()); // 노드 생성 후 spin 실행
  rclcpp::shutdown();                              // ROS2 종료
  return 0;
}
