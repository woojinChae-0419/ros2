// sub.hpp
#ifndef SUB_HPP
#define SUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

class CamSubscriber : public rclcpp::Node {
public:
    CamSubscriber();
    ~CamSubscriber();
private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};

#endif
