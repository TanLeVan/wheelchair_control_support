/**
 * This is used to calculate the minimum distance to from wheelchair to obstacles with a ros2 bag for
 * data processing after experiment
 * 
 * Date: 2025/28/5
 * Author : Le Van Tan
 */

#include "wheelchair_control_support/RectangularFootprint.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>
#include <iostream>
#include <fstream>  // Add this for file writing
#include <chrono>   // For timestamp

#define LARGE_NUM 1000000.0

class CalculateMinDistanceToObsFromRosbag : public rclcpp::Node
{
public:
    CalculateMinDistanceToObsFromRosbag(const std::string& csv_file_path)
        : Node("calculate_min_distance_to_obs_from_rosbag")
    {
        footprint_ptr_ = std::make_unique<RectangularFootprint>(footprint_width_, footprint_length_);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",rclcpp::SensorDataQoS() , std::bind(&CalculateMinDistanceToObsFromRosbag::scan_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        csv_file_.open(csv_file_path, std::ios::out | std::ios::trunc);
        csv_file_ << "timestamp,min_distance\n";
        
    }


    ~CalculateMinDistanceToObsFromRosbag()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan& scan_msg)
    {
        obstacles_points_.clear();
        for (std::size_t i = 0; i < scan_msg.ranges.size(); i+= 1) {
            float angle = scan_msg.angle_min + i * scan_msg.angle_increment;
            float range = scan_msg.ranges[i];

            if (range < collision_considered_range_ && range > scan_msg.range_min && range < scan_msg.range_max) {
                geometry_msgs::msg::Point point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;

                // Transform the point to the robot's frame
                geometry_msgs::msg::Point transformed_point;
                tf2::doTransform(point, transformed_point, tf_buffer_->lookupTransform(footprint_frame_,scan_msg.header.frame_id, tf2::TimePointZero));

                obstacles_points_.push_back(transformed_point);
            }
        }
        
        double min_distance = calculate_min_distance_to_obstacle();
        std::cout << "min distance to obstacle: " << min_distance << " meters" << std::endl;

        rclcpp::Time timestamp = scan_msg.header.stamp;
        if(is_first_timestamp_) {
            start_time = timestamp;
            is_first_timestamp_ = false; // Set the flag to false after the first timestamp
        } 
        // Write to CSV
        if (csv_file_.is_open()) {
            csv_file_ << timestamp.seconds() - start_time.seconds() << "," << min_distance << "\n";
        }
    }

    double calculate_min_distance_to_obstacle()
    {
        double min_distance = LARGE_NUM;
        footprint_ptr_->move_footprint(0.0, 0.0, 0.0); // Assuming no rotation for simplicity

        for (const auto& obs_point : obstacles_points_) {
            double distance = footprint_ptr_->distance_from_point_to_footprint(obs_point.x, obs_point.y);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        return min_distance;
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::vector<geometry_msgs::msg::Point> obstacles_points_;
    std::unique_ptr<RectangularFootprint> footprint_ptr_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double collision_considered_range_{2.0}; // meters
    std::string footprint_frame_{"base_footprint"};
    double footprint_width_{0.6}; // meters
    double footprint_length_{0.85}; // meters
    std::ofstream csv_file_;  // CSV output stream

    bool is_first_timestamp_{true}; // Flag to check if it's the first timestamp
    rclcpp::Time start_time;


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        std::cerr << "Usage: ros2 run your_package your_node_executable <csv_output_path>" << std::endl;
        return 1;
    }

    std::string csv_output_path = argv[1];

    auto node = std::make_shared<CalculateMinDistanceToObsFromRosbag>(csv_output_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}