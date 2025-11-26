// 파일명: sub.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 데이터 -> Mat (BGR)
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame.empty()){
        RCLCPP_WARN(rclcpp::get_logger("camsub"), "decoded frame empty");
        return;
    }

    // gray
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // binary (적응형이나 고정 Threshold 중 고정 사용)
    cv::Mat binary;
    const double thresh = 100.0;
    cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);

    // 창에 출력 (크기 자동 조정됨)
    cv::imshow("original", frame);
    cv::imshow("gray", gray);
    cv::imshow("binary", binary);
    cv::waitKey(1); // non-blocking
    RCLCPP_INFO(rclcpp::get_logger("camsub"), "Received Image: %s, %d x %d",
                msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    auto sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback);

    RCLCPP_INFO(node->get_logger(), "camera2-1 subscriber started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
