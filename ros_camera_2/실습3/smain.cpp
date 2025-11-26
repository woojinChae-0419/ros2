// smain.cpp
#include "rclcpp/rclcpp.hpp"
#include "sub.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
