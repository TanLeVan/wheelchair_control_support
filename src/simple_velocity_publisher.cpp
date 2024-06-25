/****
 * Test the control by velocity feature of the WHILL
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class SimpleVelocityPublisher : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist vel_cmd{};
    std::string topic;
    size_t count_;
    void timer_callback()
    {
      vel_cmd.linear.x = 0.1;
      vel_cmd.linear.y = 0;
      vel_cmd.linear.z = 0;
      vel_cmd.angular.x = 0;
      vel_cmd.angular.y = 0;
      vel_cmd.angular.z = 0;
      publisher_->publish(vel_cmd);
    }
public:
    SimpleVelocityPublisher()
    :Node("velocity_publisher"), count_(0)
    {
        this->declare_parameter("topic", "/whill/controller/cmd_vel");
        topic = this->get_parameter("topic").as_string();
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
        timer_ = this->create_wall_timer(
        1ms, std::bind(&SimpleVelocityPublisher::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleVelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}
