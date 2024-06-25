#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometryListener : public rclcpp::Node
{
public:
    OdometryListener()
        : Node("odometry_listener")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/whill/odom", 10, std::bind(&OdometryListener::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Angular Velocity: x=%.3f, y=%.3f, z=%.3f rad/s",
                    msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        RCLCPP_INFO(this->get_logger(), "Linear Velocity: x=%.3f, y=%.3f, z=%.3f rad/s",
                    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryListener>());
    rclcpp::shutdown();
    return 0;
}