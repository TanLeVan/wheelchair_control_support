#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "wheelchair_control_support/EgocircleOccupancyGrid.hpp"
#include <Eigen/Dense>

class EgocircleNode : public rclcpp::Node {
public:
    EgocircleNode() : Node("egocircle_node"), map_(270, 30, 2.), has_previous_odom_(false) {
        // Subscriber for LaserScan messages
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&EgocircleNode::laser_scan_callback, this, std::placeholders::_1)
        );

        // Subscriber for Odometry messages
        // odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom",
        //     rclcpp::SensorDataQoS(),
        //     std::bind(&EgocircleNode::odom_callback, this, std::placeholders::_1)
        // );

        // Publisher for visualizing the occupancy map
        map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/egocircle_map", 10);

        // Timer for periodic visualization
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { map_.visualize_map(map_publisher_); }
        );
    }

private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan& msg) {
        RCLCPP_INFO(this->get_logger(), "Processing LaserScan with %zu ranges", msg.ranges.size());
        map_.update_map_with_lidar_scan(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry& msg) {
        RCLCPP_INFO(this->get_logger(), "Processing Odometry message");

        // Compute relative SE(2) transformation if previous odometry exists
        if (has_previous_odom_) {
            // Extract current pose
            double x_curr = msg.pose.pose.position.x;
            double y_curr = msg.pose.pose.position.y;
            double yaw_curr = std::atan2(
                2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                       msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                             msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
            );

            // Extract previous pose
            double x_prev = previous_odom_.pose.pose.position.x;
            double y_prev = previous_odom_.pose.pose.position.y;
            double yaw_prev = std::atan2(
                2.0 * (previous_odom_.pose.pose.orientation.w * previous_odom_.pose.pose.orientation.z +
                       previous_odom_.pose.pose.orientation.x * previous_odom_.pose.pose.orientation.y),
                1.0 - 2.0 * (previous_odom_.pose.pose.orientation.y * previous_odom_.pose.pose.orientation.y +
                             previous_odom_.pose.pose.orientation.z * previous_odom_.pose.pose.orientation.z)
            );

            // Compute relative transformation
            Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
            double dx = x_curr - x_prev;
            double dy = y_curr - y_prev;
            double dyaw = yaw_curr - yaw_prev;

            transform(0, 2) = dx * std::cos(yaw_prev) + dy * std::sin(yaw_prev); // Transform translation
            transform(1, 2) = -dx * std::sin(yaw_prev) + dy * std::cos(yaw_prev);
            transform(0, 0) = std::cos(dyaw);
            transform(0, 1) = -std::sin(dyaw);
            transform(1, 0) = std::sin(dyaw);
            transform(1, 1) = std::cos(dyaw);

            // Apply the transformation to the map
            map_.transform_map(transform);
        }

        // Update previous odometry
        previous_odom_ = msg;
        has_previous_odom_ = true;
    }

    EgocircleOccupancyMap map_;
    nav_msgs::msg::Odometry previous_odom_;
    bool has_previous_odom_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EgocircleNode>());
    rclcpp::shutdown();
    return 0;
}
