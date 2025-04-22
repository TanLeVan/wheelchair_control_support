#ifndef CIRCULAR_FOOTPRINT_
#define CIRCULAR_FOOTPRINT_

#include "AbstractFootprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class CircularFootprint : public AbstractFootprint
{
public:
    CircularFootprint(double center_x, double center_y, double radius)
        : center_x_{center_x}, center_y_{center_y}, radius_{radius}
    {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }

    CircularFootprint(double radius)
        : CircularFootprint{0,0, radius}
    {
    }

    bool check_point_inside_footprint (const double x, const double y) override
    /*true: inside
      false: not inside
    */
    {
        return (hypot((x - center_x_), (y-center_y_)) <= radius_);
    }

    double distance_from_point_to_footprint(const double x, const double y) override
    {
        return std::max(0.0, hypot((x - center_x_), (y - center_y_)) - radius_);
    }

    void move_footprint(const double x, const double y, const double ) override
    {
        center_x_ = x;
        center_y_ = y;
        
    }
    
    visualization_msgs::msg::Marker generate_footprint_marker(std::string frame_id, int marker_id) override
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = frame_id;
        marker.header.stamp = clock_->now();
        marker.ns = "circle";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // Circle properties
        marker.pose.position.x = center_x_;
        marker.pose.position.y = center_y_;
        marker.scale.x = radius_/50;  // Line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        // Generate points for the circle
        int num_points = 50;
        for (int i = 0; i <= num_points; ++i) {
            float angle = 2 * M_PI * i / num_points;
            geometry_msgs::msg::Point p;
            p.x = radius_ * cos(angle);
            p.y = radius_ * sin(angle);
            p.z = 0;
            marker.points.push_back(p);
        }
        return marker;
    }
private:
    double center_x_{0};
    double center_y_{0};
    double radius_;
    std::shared_ptr<rclcpp::Clock> clock_;
};

#endif