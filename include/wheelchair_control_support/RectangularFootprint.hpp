#ifndef RECTANGULAR_FOOTPRINT_
#define RECTANGULAR_FOOTPRINT_

#include "AbstractFootprint.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

class RectangularFootprint : public AbstractFootprint
{
public:
    RectangularFootprint(){}
    /**
     * Constructor that allow to set all parameters
     */
    RectangularFootprint(double center_x, double center_y, double theta, double height, double width)
    : center_x_{center_x}, center_y_{center_y}, theta_{theta}, height_{height}, width_{width}
    {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }
    /**
     * Construct rectangular footprint with specific width (x-axis) and height (y-axis)
     */
    RectangularFootprint(double height, double width)
    : RectangularFootprint(0, 0, 0, height, width)
    {}

    /*true: inside
      false: not inside
    */
    void move_footprint(const double x, const double y, const double theta)
    {
        center_x_ = x;
        center_y_ = y ;
        theta_ = theta;
    }

    bool check_point_inside_footprint(const double x, const double y)
    {
        /**Transform the coordinate of point from original coordinate to the coordinate fixed at the center of the footprint**/
        double x_wheelchair = x*std::cos(theta_)+y*std::sin(theta_) - center_x_*std::cos(theta_) - center_y_*std::sin(theta_);
        double y_wheelchair = -x*std::sin(theta_)+y*std::cos(theta_) + center_x_*std::sin(theta_) - center_y_*std::cos(theta_);

        /*Check if point is inside rectangular by comparing the position of point wrt to the rectangular footprint to the  width and height*/
        if(x_wheelchair >= -width_/2 && x_wheelchair<=width_/2 && y_wheelchair >= -height_/2 && y_wheelchair <= height_/2)
            return true;
        
        return false;
    }

    double distance_from_point_to_footprint(const double x, const double y)
    {
        double x_wheelchair = x*std::cos(theta_)+y*std::sin(theta_) - center_x_*std::cos(theta_) - center_y_*std::sin(theta_);
        double y_wheelchair = -x*std::sin(theta_)+y*std::cos(theta_) + center_x_*std::sin(theta_) - center_y_*std::cos(theta_);

          // Calculate the distance to the nearest edge in both directions
        double dx = std::abs(x_wheelchair) - width_/2;  // Calculate distance from left or right edge
        double dy = std::abs(y_wheelchair) - height_/2; // Calculate distance from top or bottom edge
        double distance{};
        // If the point is inside the footprint, return negative distances
        if (dx < 0 && dy < 0)
        {
            distance = -std::sqrt(dx * dx + dy * dy);
        }
        else{
            distance = std::sqrt(dx * dx + dy * dy);
        }

        // Return the Euclidean distance to the nearest edge (non-zero if outside the footprint)
        return distance;
    }

    visualization_msgs::msg::Marker generate_footprint_marker(std::string frame_id, int marker_id)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = frame_id;
        marker.header.stamp = clock_->now();
        marker.ns = "rectangular";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // Rectangle properties
        marker.pose.position.x = center_x_;
        marker.pose.position.y = center_y_;
        marker.pose.orientation.z = std::sin(theta_/2);
        marker.pose.orientation.w = std::cos(theta_/2);
        marker.scale.x = height_/50;  // Line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        geometry_msgs::msg::Point p1;
        p1.x = width_/2;
        p1.y = height_/2;
        marker.points.push_back(p1);
        geometry_msgs::msg::Point p2;
        p2.x = -width_/2;
        p2.y = height_/2;
        marker.points.push_back(p2);
        geometry_msgs::msg::Point p3;
        p3.x = -width_/2;
        p3.y = -height_/2;
        marker.points.push_back(p3);
        geometry_msgs::msg::Point p4;
        p4.x = width_/2;
        p4.y = -height_/2;
        marker.points.push_back(p4);
        marker.points.push_back(p1);
        return marker;
    }

    double get_height() const
    {
        return height_;
    }
private:
    double center_x_{0};
    double center_y_{0};
    double theta_{0};
    double height_;
    double width_;
    std::shared_ptr<rclcpp::Clock> clock_;
};

#endif
