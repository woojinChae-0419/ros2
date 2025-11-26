// sub.cpp
#include "sub.hpp"

CamSubscriber::CamSubscriber(): Node("camsub_class")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, std::bind(&CamSubscriber::callback, this, std::placeholders::_1));
}

CamSubscriber::~CamSubscriber(){}

void CamSubscriber::callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame.empty()){
        RCLCPP_WARN(this->get_logger(), "decoded frame empty");
        return;
    }
    cv::imshow("class_original", frame);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received: %d x %d", frame.cols, frame.rows);
}
