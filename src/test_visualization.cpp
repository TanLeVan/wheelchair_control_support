#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "wheelchair_control_support/AbstractFootprint.hpp"
#include "wheelchair_control_support/CircularFootprint.hpp"
#include <memory>

using namespace std::chrono_literals;
class FootprintVisualizer : public rclcpp::Node
{
public: 
    FootprintVisualizer()
        : Node("footprint_visualizer")
    {
        footprint_ptr = std::make_unique<CircularFootprint>(0, 0, 1);
        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("footprint_marker", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&FootprintVisualizer::timer_callback, this));
    }
private:
    void timer_callback()
    {
        pub_->publish(footprint_ptr->generate_footprint_marker("map"));
    }
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    std::unique_ptr<AbstractFootprint> footprint_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argv, char* argc[])
{
    rclcpp::init(argv, argc);
    rclcpp::spin(std::make_shared<FootprintVisualizer>());
    rclcpp::shutdown();
    return 0;
}