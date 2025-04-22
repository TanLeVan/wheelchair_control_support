#include "wheelchair_control_support/Gap.hpp"
#include "wheelchair_control_support/FindGap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>


class FindGapNode : public rclcpp::Node
{
public:
    FindGapNode() : Node("find_gap")
    {
        std::cout << "Initialize Find Gap Node " << std::endl;
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "filtered_scan", rclcpp::SensorDataQoS(), std::bind(&FindGapNode::scan_callback, this, std::placeholders::_1));
        gap_visualize_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "gap_visualize", 1
        );
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
    // RCLCPP_INFO(this->get_logger(), "Received scan: [angle_min: %f, angle_max: %f, angle_increment: %f]",
    //             msg->angle_min, msg->angle_max, msg->angle_increment);

    // Detect gaps
    observed_gaps.clear();
    gap_finder.find_gap_from_scan(*msg, observed_gaps);


    // Prepare marker array for visualization
    visualization_msgs::msg::MarkerArray marker_array;

    // Marker for clearing previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Add markers for the detected gaps
    int id = 0;
    // for (const auto& gap : observed_gaps) 
    // {
    //     visualization_msgs::msg::Marker marker;
    //     marker.header.frame_id = msg->header.frame_id;
    //     marker.header.stamp = this->get_clock()->now();
    //     marker.ns = "gaps";
    //     marker.id = id++;
    //     marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //     marker.action = visualization_msgs::msg::Marker::ADD;

    //     // Set color based on gap type
    //     if (gap.is_radial()) {
    //         marker.color.r = 0.0;
    //         marker.color.g = 1.0; // Green for radial gaps
    //         marker.color.b = 0.0;
    //     } else {
    //         marker.color.r = 1.0; // Red for non-radial gaps
    //         marker.color.g = 0.0;
    //         marker.color.b = 0.0;
    //     }
    //     marker.color.a = 1.0; // Fully opaque

    //     marker.scale.x = 0.05; // Line width

    //     // Compute Cartesian coordinates for the left and right points
    //     geometry_msgs::msg::Point p_left, p_right;

    //     double angle_left = msg->angle_min + gap.get_l_idx() * msg->angle_increment;
    //     double angle_right = msg->angle_min + gap.get_r_idx() * msg->angle_increment;

    //     p_left.x = gap.get_l_dist() * cos(angle_left);
    //     p_left.y = gap.get_l_dist() * sin(angle_left);
    //     p_left.z = 0.0; // Assuming a 2D scan

    //     p_right.x = gap.get_r_dist() * cos(angle_right);
    //     p_right.y = gap.get_r_dist() * sin(angle_right);
    //     p_right.z = 0.0; // Assuming a 2D scan

    //     // Add points to the marker
    //     marker.points.push_back(p_left);
    //     marker.points.push_back(p_right);
    //     marker_array.markers.push_back(marker);
    // }

    // Add markers for the merged gap
    // gap_visualize_publisher_->publish(gap_finder.visualize_gaps(*msg, observed_gaps, id));
    gap_finder.merge_gap(*msg, observed_gaps);
    id = 100;
    gap_visualize_publisher_->publish(gap_finder.visualize_gaps(*msg, observed_gaps, id, 100));
}

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_visualize_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    FindGap gap_finder;
    std::vector<Gap> observed_gaps;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindGapNode>());
    return 0;
}