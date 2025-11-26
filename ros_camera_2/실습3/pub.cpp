// pub.cpp
#include "pub.hpp"
#include "cv_bridge/cv_bridge.h"

std::string src = "nvarguscamerasrc sensor-id=0 ! \
video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
width=(int)640, height=(int)360, format=(string)BGRx ! \
videoconvert ! video/x-raw, format=(string)BGR ! appsink";

CamPublisher::CamPublisher(): Node("campub_class")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);

    cap_.open(src, cv::CAP_GSTREAMER);
    if(!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera in CamPublisher");
        throw std::runtime_error("camera open failed");
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&CamPublisher::timer_callback, this)
    );
}

CamPublisher::~CamPublisher()
{
    if(cap_.isOpened()) cap_.release();
}

void CamPublisher::timer_callback()
{
    cv::Mat frame;
    cap_ >> frame;
    if(frame.empty()){
        RCLCPP_WARN(this->get_logger(), "frame empty");
        return;
    }
    auto msg = cv_bridge::CvImage(hdr_, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}
