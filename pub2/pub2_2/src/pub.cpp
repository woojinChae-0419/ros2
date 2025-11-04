#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pub2_2_node");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("vector_topic", qos);

    geometry_msgs::msg::Vector3 msg;

    while (rclcpp::ok())
    {
        std::cout << "Enter 3 float values (x y z): ";
        std::cin >> msg.x >> msg.y >> msg.z;

        RCLCPP_INFO(node->get_logger(), "Publishing: x=%.2f, y=%.2f, z=%.2f",
                    msg.x, msg.y, msg.z);
        pub->publish(msg);
    }

    rclcpp::shutdown();
    return 0;
}
