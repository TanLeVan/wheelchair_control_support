#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class SimpleJoy : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    sensor_msgs::msg::Joy joy_;
    void subscriber_callback(const sensor_msgs::msg::Joy &msg)
    {
        joy_ = msg;
    }
    void timer_callback()
    {
      pub_->publish(joy_);
    }
public:
    SimpleJoy()
    :Node("joy_control")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/controller/joy", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/whill/states/joy", rclcpp::SensorDataQoS(),
                                                                       std::bind(&SimpleJoy::subscriber_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
        1ms, std::bind(&SimpleJoy::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleJoy>());
  rclcpp::shutdown();
  return 0;
}
