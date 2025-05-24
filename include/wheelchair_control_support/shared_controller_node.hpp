#ifndef WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP
#define WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "wheelchair_control_support/msg/gap.hpp"
#include "wheelchair_control_support/NoiseGenerator.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_shared_mppi_controller/controller.hpp"


using nav2_shared_mppi_controller::MPPISharedController;

class SharedControllerNode : public rclcpp::Node
{

public:
    struct WhillDynamic
    {
        double max_linear_vel_{0};
        double min_linear_vel_{0};
        double max_yaw_rate_{0};
        double max_acceleration_{0};
        double max_deceleration_{0};
        double max_yaw_acceleration_{0};
    };

    SharedControllerNode();
    ~SharedControllerNode() override;
    void configure(void);
    void activate(void);

private:
    void main_process(void);
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    void joystick_callback(const sensor_msgs::msg::Joy &msg);
    void gap_callback(const wheelchair_control_support::msg::Gap &msg);
    void path_callback(const nav_msgs::msg::Path &msg);


    /**
     * @brief Convert joystick input to proportional velocity. Can add noise
     * @param joy The joystick input
     * @return The calculated velocity
     */
    geometry_msgs::msg::Twist calculate_velocity_from_joy(sensor_msgs::msg::Joy & joy);

    /**
     * @brief Calculate corresponding joy output for velocity
     */
    sensor_msgs::msg::Joy calculate_joy_from_velocity(const geometry_msgs::msg::Twist & vel);

    /**
     * @brief Convert gap to pose
     * @param gap The gap message
     * @return The converted pose
     * @details The gap is converted to a pose with the middle point of the gap as the position and the yaw angle is set perpendicular to gap.
     */
    geometry_msgs::msg::Pose convert_gap_to_pose(const wheelchair_control_support::msg::Gap & gap);

    void user_trajectory_visualization();

        
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<NoiseGenerator> noise_generator_;

    //Controller
    std::shared_ptr<nav2_shared_mppi_controller::MPPISharedController> mppi_controller_;

    // Tf stuff
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriber and publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //Sensor QoS should be use
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; //Receive user input
    rclcpp::Subscription<wheelchair_control_support::msg::Gap>::SharedPtr gap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_; //Using joy control as output to compensate for jerky motion of the wheelchair
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_visualizer_pub_; //For visualizing trajectory

    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::Joy joystick_;
    wheelchair_control_support::msg::Gap observed_gap_; //Observed gap with highest confidence
    nav_msgs::msg::Path path_;

    /*Check if new information from subscriber is receieved*/
    bool is_odom_updated_{false};
    bool is_joystick_updated_{false};
    bool is_gap_updated_{false};

    // Parameters
    bool joystick_noise_{false};                // Add noise to joystick input
    double noise_freq_{5};                  // Frequency of noise generation
    double noise_std_{0.4}; 
    WhillDynamic whill_dynamic_; // Whill dynamic parameters
    double period_{0.1}; 
    
};


#endif //WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP