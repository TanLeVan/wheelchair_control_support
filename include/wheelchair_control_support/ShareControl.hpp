/**
 * Author: Le Van Tan
 * Email: levantann321@gmail.com / le.van.tan.t4@dc.tohoku.ac.jp
 * Date: 2024/5/23
 * **/

#ifndef SHARE_CONTROL_
#define SHARE_CONTROL_
#include <vector>
#include <utility>

#include "wheelchair_control_support/AbstractFootprint.hpp"
#include "wheelchair_control_support/msg/gap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class ShareControl : public rclcpp::Node
{
public:
    class State
    {
        public: 
            State(){};
            State(const double x, const double y, const double yaw, const double velocity, const double yaw_rate);
            double x_{0};
            double y_{0};
            double yaw_{0};
            double velocity_{0};
            double yaw_rate_{0};

            State next_state(const double period)
            {
                State next_state;
                next_state.yaw_ = yaw_ + yaw_rate_*period;
                next_state.x_ = x_ + velocity_ * std::cos(yaw_ + yaw_rate_*period/2) * period;
                next_state.y_ = y_ + velocity_ * std::sin(yaw_ + yaw_rate_*period/2) * period;
                next_state.velocity_ = velocity_;
                next_state.yaw_rate_ = yaw_rate_;
                return next_state;
            }
    };
    struct Window
    { 
        double min_velocity_;
        double max_velocity_;
        double min_yaw_rate_;
        double max_yaw_rate_;
    };
    struct WhillDynamic
    {
        double max_linear_vel_{0};
        double min_linear_vel_{0};
        double max_yaw_rate_{0};
        double max_acceleration_{0};
        double max_deceleration_{0};
        double max_yaw_acceleration_{0};
    };

    ShareControl();
    void main_process(void);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan &msg);
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    void joystick_callback(const sensor_msgs::msg::Joy &msg);
    void gap_callback(const wheelchair_control_support::msg::Gap &msg);

    /**
     * @brief From scan message, calculate the position of obstacles. Update to obs_lists_
     * @param scan the scan message
     * @return The distance from robot footprint to the nearest obstacle
     * **/
    void scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan);

    /**
     * Calculate corresponding velocity from WHILL joystick input
     * Joy input is joystick_ member variable
    **/
    geometry_msgs::msg::Twist calculate_velocity_from_joy(void);

    /**
     * Calculate corresponding virtual joystick from velocity
     */
    sensor_msgs::msg::Joy calculate_joy_from_velocity(geometry_msgs::msg::Twist  vel);

    /**
     * Generate trajectory from linear and angular velocity for sim_time_ in the future
     * **/
    std::vector<State> generate_trajectory(const double linear_vel, const double yaw_rate);
    
    /**
     * Evaluate the cost of a given trajectory
     * **/
    void evaluate_trajectory(const std::vector<State> &traj);

    /**
     * Cost function to evaluate a velocity pair
     */
    double calculate_vel_pair_cost(const double linear_vel, 
                            const double yaw_rate, 
                            const wheelchair_control_support::msg::Gap &gap,
                            const double linear_vel_user,
                            const double yaw_rate_user,
                            const std::vector<State>& traj);

    /**
     * Check for collision of trajectory
     * true: collide
     * false: not collide
     * **/
    bool check_for_colllision(const std::vector<State> &traj);

    double closest_distance_to_obstacle(const State& state);

    /**
     * Calculate the next state of the robot after one time step
     * @param state reference to the current state of the robot
     * @param linear_vel the assumed linear velocity
     * @param yaw_rate the assumed yaw rate
     * return the next state of the robot assuming linear_vel and yaw_rate. The next state overwrite the state parameter.
     * **/
    // void motion(State &state,const double linear_vel,const double yaw_rate);

    /**
     * Visualize trajectory with footprint
     * **/
    void trajectory_visualization(const std::vector<State> &traj);

    /**
     * User input trajectory visualization
     * This function draw trajectory as a line , without footprint
     */
    void user_trajectory_visualization(const std::vector<State> &traj);

    /**
     * Calculate the dynamic window from the current velocity
     * **/
    Window cal_dynamic_window(void);

    /**
     * Create limited pair of linear angular velocity from the dynammic window 
    **/
    std::vector<std::pair<double, double>> discretize_dynamic_window(const Window &window);

    /***
     * Make command velocity to 0
     */
    void reset_command_vel(void);

    /**
     * Publish velocity as discrete velocity point that increase linearly until the desire velocity is reached.
     * **/
    void publish_vel_smooth(int no_step);

private:
    sensor_msgs::msg::LaserScan scan_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::Joy joystick_;
    geometry_msgs::msg::Twist cmd_vel_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_; //Sensor QoS should be use
    rclcpp::Subscription<nav_msgs::msg::Odometry >::SharedPtr odom_subscriber_; //Sensor QoS should be use
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_; //Sensor QoS should be use
    rclcpp::Subscription<wheelchair_control_support::msg::Gap>::SharedPtr gap_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obs_list_pub_; // For visualizing obstacle
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_visualizer_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  user_traj_visualizer_pub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::unique_ptr<AbstractFootprint> footprint_ptr_;
    WhillDynamic whill_dynamic_;
    geometry_msgs::msg::PoseArray obs_list_; /*List of observed obstacle*/
    wheelchair_control_support::msg::Gap observed_gap_; //Observed gap with highest confidence

    /*Check if new information from subscriber is receieved*/
    bool is_scan_updated_{false};
    bool is_odom_updated_{false};
    bool is_joystick_updated_{false};
    bool is_gap_updated_{false};

    /*Parameter to change the behavior of the DWA algorithm*/
    std::string robot_frame_{"base_footprint"}; //We consider everything with respect to robot_frame_
    double predict_time_{3};                   //Time to predict the trajectory (in second)
    double predict_timestep_{0.2};              // Time step between each state in a trajectory
    double scan_to_obs_range_{3};               /*All point from scan within this range is considered
                                                obstacle. Point outside the range will be ignored*/
    int skip_point_{2};                         //How many consercutive points is needed to register one obstacle ?? For sparsing the pointcloud
    double period_{0.5};                        // Period of one loop of execution. (1/Frequency)
    int linear_vel_sample_size_{5};              // How many discrete linear vel point within dynamic window will be considered
    int yaw_rate_sample_size_{10};                // How many discrete yaw rate point within dynamic window will be considered
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif