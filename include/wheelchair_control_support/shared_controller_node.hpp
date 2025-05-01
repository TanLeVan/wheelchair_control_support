#ifndef WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP
#define WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "wheelchair_control_support/msg/gap.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_shared_mppi_controller/controller.hpp"
#include "wheelchair_smac_planner/smac_planner_hybrid.hpp"


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
    geometry_msgs::msg::PoseStamped convert_gap_to_pose(const wheelchair_control_support::msg::Gap & gap);

    bool validatePath(const nav_msgs::msg::Path &path);

    void user_trajectory_visualization();

    void computePathToPose(const geometry_msgs::msg::PoseStamped &goal, nav_msgs::msg::Path& path);    

    // The controller needs a costmap node
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

    //Controller
    std::shared_ptr<nav2_shared_mppi_controller::MPPISharedController> mppi_controller_;

    //Global Planner
    std::shared_ptr<wheelchair_smac_planner::SmacPlannerHybrid> smac_planner_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;


    // Subscriber and publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //Sensor QoS should be use
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; //Receive user input
    rclcpp::Subscription<wheelchair_control_support::msg::Gap>::SharedPtr gap_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_; //Using joy control as output to compensate for jerky motion of the wheelchair
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_visualizer_pub_; //For visualizing trajectory
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::Joy joystick_;
    wheelchair_control_support::msg::Gap observed_gap_; //Observed gap with highest confidence
    /*Check if new information from subscriber is receieved*/
    bool is_odom_updated_{false};
    bool is_joystick_updated_{false};
    bool is_gap_updated_{false};

    //For experiment. Add noise to joystick input to simulate patient with hand tremor
    bool joystick_noise_{false};                // Add noise to joystick input

    WhillDynamic whill_dynamic_; // Whill dynamic parameters
    double period_{0.1};                    
    rclcpp::TimerBase::SharedPtr timer_;
};


#endif //WHEELCHAIR_CONTROL_SUPPORT_SHARED_CONTROLLER_NODE_HPP