#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PubNode : public rclcpp::Node
{
public:
  PubNode() : Node("node_pub1_1"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_pub1_1", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PubNode::timer_callback, this));  // 100ms마다 호출
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello world! count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubNode>());
  rclcpp::shutdown();
  return 0;
}
