#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>

class IntSubscriber : public rclcpp::Node
{
public:
  IntSubscriber()
  : Node("node_sub1_2")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "int_topic", 
      10,
      std::bind(&IntSubscriber::topic_callback, this, std::placeholders::_1)
    );
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntSubscriber>());
  rclcpp::shutdown();
  return 0;
}
