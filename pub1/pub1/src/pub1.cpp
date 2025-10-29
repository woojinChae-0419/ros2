#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node_pub1");

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  auto pub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1", qos_profile);

  std_msgs::msg::Int32 message;
  message.data = 0;

  rclcpp::WallRate loop_rate(1.0); // 1 Hz
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);
    pub->publish(message);
    message.data += 1;
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
