#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
  VideoPublisher()
  : Node("video_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video/image_raw", 10);
    
    // [사용자 경로 확인 완료] linux 사용자에 맞게 설정된 경로입니다.
    video_path_ = "/home/linux/simulation/5_lt_cw_100rpm_out.mp4"; 
    
    cap_.open(video_path_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video file: %s", video_path_.c_str());
      rclcpp::shutdown();
    }

    timer_ = this->create_wall_timer(
      33ms, std::bind(&VideoPublisher::timer_callback, this)); // 약 30fps
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      // 영상이 끝나면 다시 처음으로 되감기 (반복 재생)
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      cap_ >> frame;
    }

    // PDF 요구사항에 맞춰 해상도 리사이즈 (640x360)
    cv::resize(frame, frame, cv::Size(640, 360));

    // ROS 메시지로 변환 및 발행
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  std::string video_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}