// pmain.cpp
#include "rclcpp/rclcpp.hpp"
#include "pub.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
