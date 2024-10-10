#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class FindGap : public rclcpp::Node
{
public:
    FindGap() : Node("laser_scan_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&FindGap::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received scan: [angle_min: %f, angle_max: %f]", msg->angle_min, msg->angle_max);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindGap>());
    rclcpp::shutdown();
    return 0;
}
