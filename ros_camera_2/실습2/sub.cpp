// 파일명: sub.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

cv::VideoWriter writer;
bool writer_opened = false;

void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame.empty()){
        RCLCPP_WARN(rclcpp::get_logger("camsub"), "decoded frame empty");
        return;
    }

    if(!writer_opened){
        // 코덱과 파일명은 환경에 따라 다를 수 있음. 'mp4v' 또는 'avc1' 시도
        int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
        double fps = 30.0;
        cv::Size sz(frame.cols, frame.rows);
        writer.open("output.mp4", fourcc, fps, sz, true);
        if(!writer.isOpened()){
            RCLCPP_ERROR(rclcpp::get_logger("camsub"), "Failed to open VideoWriter");
        } else {
            writer_opened = true;
            RCLCPP_INFO(rclcpp::get_logger("camsub"), "VideoWriter opened: output.mp4");
        }
    }

    // 화면 출력
    cv::imshow("recording", frame);
    cv::waitKey(1);

    // 파일 저장
    if(writer_opened){
        writer.write(frame);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_rec");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    auto sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback);

    RCLCPP_INFO(node->get_logger(), "camera2-2 subscriber started. Press Ctrl+C to stop and finalize file.");
    rclcpp::spin(node);

    // shutdown 후 writer 해제
    if(writer_opened){
        writer.release();
        RCLCPP_INFO(rclcpp::get_logger("camsub"), "VideoWriter released, file saved.");
    }

    rclcpp::shutdown();
    return 0;
}
