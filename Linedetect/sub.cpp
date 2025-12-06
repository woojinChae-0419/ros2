#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono> 

using std::placeholders::_1;
using namespace cv;
using namespace std;

class LineDetector : public rclcpp::Node
{
public:
  LineDetector()
  : Node("line_detector")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video/image_raw", 10, std::bind(&LineDetector::topic_callback, this, _1));
    
    // 초기 라인 위치는 영상의 중심(320)으로 가정
    prev_line_x_ = 320; 
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 1. 시간 측정 시작
    auto start_time = std::chrono::steady_clock::now();

    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
    // 2. 전처리 (Preprocessing)
    // 2-1. ROI 선정: 하단 1/4 (y=270~360)
    int roi_y = 270;
    int roi_h = 90;
    cv::Rect roi_rect(0, roi_y, 640, roi_h);
    cv::Mat roi = img(roi_rect); 

    // 2-2. 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // 2-3. 밝기 보정 (목표 밝기 100)
    cv::Scalar mean_val = cv::mean(gray);
    int brightness_diff = 100 - (int)mean_val[0];
    gray = gray + brightness_diff;

    // 2-4. 이진화 (Thresholding)
    cv::Mat binary;
    // 임계값 160. 이 값을 조정하여 검출 성능을 개선할 수 있습니다.
    cv::threshold(gray, binary, 160, 255, cv::THRESH_BINARY);

    // 이진화된 흑백 영상에 색깔을 그릴 수 있도록 BGR(3채널)로 변환
    cv::Mat binary_color;
    cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);

    // 3. 라인 검출 (Labeling)
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

    int best_line_idx = -1;
    double min_dist = 99999.0;
    
    // 라인 후보들을 순회하며 진짜 라인 찾기
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int cx = (int)centroids.at<double>(i, 0);
        // int cy = (int)centroids.at<double>(i, 1); // 사용되지 않는 변수
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        // 3-1. 노이즈 제거 (면적 필터링)
        if (area < 50 || area > 5000) continue;

        // 모든 후보는 이진화 영상(binary_color)에 파란색 박스 그리기
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(255, 0, 0), 1);

        // 3-2. 트래킹: 이전 라인 위치와 가장 가까운 객체 선택
        double dist = std::abs(cx - prev_line_x_);
        if (dist < min_dist) {
            min_dist = dist;
            best_line_idx = i;
        }
    }

    int error = 0;
    if (best_line_idx != -1) {
        // 라인을 찾은 경우
        int cx = (int)centroids.at<double>(best_line_idx, 0);
        // int cy = (int)centroids.at<double>(best_line_idx, 1); // 사용되지 않는 변수
        int x = stats.at<int>(best_line_idx, cv::CC_STAT_LEFT);
        int y = stats.at<int>(best_line_idx, cv::CC_STAT_TOP);
        int w = stats.at<int>(best_line_idx, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(best_line_idx, cv::CC_STAT_HEIGHT);

        // 최종 라인은 이진화 영상(binary_color)에 빨간색 박스와 점으로 표시
        cv::rectangle(binary_color, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2);
        cv::circle(binary_color, cv::Point(cx, y + h/2), 4, cv::Scalar(0, 0, 255), -1);

        // 4. 에러 계산
        error = 320 - cx; // 로봇 중심(320) - 라인 중심(cx)
        prev_line_x_ = cx; // 다음 주기를 위해 현재 위치 저장
    } else {
        // 라인을 못 찾은 경우
    }

    // 5. 처리 시간 계산 및 출력
    auto end_time = std::chrono::steady_clock::now();
    float processing_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    // 터미널 출력
    std::cout << "error: " << error << ", time: " << processing_time << " ms" << std::endl;

    // 화면 출력: 원본 영상 전체(640x360), 이진화된 결과(결과 포함) 분리 출력
    cv::imshow("Original Full Image", img); 
    cv::imshow("Binary Result View", binary_color); 

    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int prev_line_x_;
};

int main(int argc, char * argv[])
{
  // 오타 수정: rclcpp::init(argc, argc) -> rclcpp::init(argc, argv)
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetector>());
  rclcpp::shutdown();
  return 0;
}