#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

void move_turtle(rclcpp::Node::SharedPtr node,
                 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 1.0;
    cmd.angular.z = 1.0;
    RCLCPP_INFO(node->get_logger(), "cmd_vel: linear=%.2f, angular=%.2f",
                cmd.linear.x, cmd.angular.z);
    pub->publish(cmd);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pub2_3_node");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos);

    std::function<void()> fn = std::bind(move_turtle, node, pub);
    auto timer = node->create_wall_timer(100ms, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
