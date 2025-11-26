// pub.hpp
#ifndef PUB_HPP
#define PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

class CamPublisher : public rclcpp::Node {
public:
    CamPublisher();
    ~CamPublisher();
private:
    void timer_callback();
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std_msgs::msg::Header hdr_;
};

#endif
