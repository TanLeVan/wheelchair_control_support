#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>

using namespace std::chrono_literals;
class CalculateAcceleration : public rclcpp::Node
{
public:
    CalculateAcceleration()
        : Node("calculate_acceleration")
    {   
        this->declare_parameter("topic", "/whill/odom");
        topic = this->get_parameter("topic").as_string();
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(topic, rclcpp::SensorDataQoS() ,std::bind(&CalculateAcceleration::callback, this, std::placeholders::_1));
    }
private:
    void callback(const nav_msgs::msg::Odometry &msg)
    {
        rclcpp::Time time1(msg.header.stamp);
        rclcpp::Time time2(odom_msg.header.stamp);
        rclcpp::Duration duration  = time1 - time2;
        float linear_displacement  = msg.twist.twist.linear.x - odom_msg.twist.twist.linear.x;
        float angular_displacement = msg.twist.twist.angular.z - odom_msg.twist.twist.angular.z;
        std::cout << "Linear acceleration   " << std::round(linear_displacement/duration.seconds()*1000)/1000 << std::endl;
        std::cout << "Angular acceleration  " << std::round(angular_displacement/duration.seconds()*1000)/1000 << std::endl;
        std::cout << std::endl;
        odom_msg = msg;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::string topic;
    nav_msgs::msg::Odometry odom_msg;
};

int main(int argc , char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalculateAcceleration>());
    rclcpp::shutdown();
    return 0;
}
