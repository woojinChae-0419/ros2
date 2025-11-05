#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class SubNode : public rclcpp::Node
{
public:
  SubNode() : Node("node_sub1_1")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_pub1_1",
      10,
      std::bind(&SubNode::topic_callback, this, std::placeholders::_1)
    );
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubNode>());
  rclcpp::shutdown();
  return 0;
}
