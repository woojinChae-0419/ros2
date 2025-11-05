#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class IntPublisher : public rclcpp::Node
{
public:
  IntPublisher()
  : Node("node_pub1_2"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
    // 50ms마다 콜백 실행
    timer_ = this->create_wall_timer(50ms, std::bind(&IntPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntPublisher>());
  rclcpp::shutdown();
  return 0;
}
