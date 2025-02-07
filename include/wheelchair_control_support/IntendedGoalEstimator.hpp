#ifndef INTENDED_GOAL_ESTIMATOR_
#define INTENDED_GOAL_ESTIMATOR_

#include <vector>
#include <deque>
#include <utility>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// For synchronization of joystick and odom
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wheelchair_control_support/msg/gap.hpp"
#include "wheelchair_control_support/Gap.hpp"
#include "wheelchair_control_support/FindGap.hpp"

class GapIntentionEstimator : public rclcpp::Node
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Joy, nav_msgs::msg::Odometry> SyncPolicy;
public:
    class State
    {
    public:
        double distance_to_goal_;
        double angular_distance_to_goal_;
    };

    //Class to describe action of the ser
    class Action
    {
    public:
        double linear_vel_;
        double angular_vel_;
    };   

    struct WhillDynamic
    {
        double max_linear_vel_{0.3};
        double min_linear_vel_{-0.3};
        double max_yaw_rate_{1.6};
        double max_acceleration_{1.};
        double max_deceleration_{2.};
        double max_yaw_acceleration_{2.5};
    };

    //Class to describe state of the wheelchair 
    // can be changed later if needed?

    GapIntentionEstimator();

    /**
     * Read from a csv file, load value function into 2d vector
     */
    void get_value_function(const std::string& value_function_filename, std::vector<std::vector<double>>& value_function);

    /**
     * This is just to verify the correctness of get_value_function
     */
    void save_value_function(const std::string& output_filename, const std::vector<std::vector<double>>& value_function);

    void setup_subscriber(const std::string& joy_topic, const std::string& odom_topic, const std::string& scan_topic);

    void joy_odom_callback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg);

    /**
     * Calculate the state trajectory for a gap from raw trajectory
     * @arg: raw_trajectory: 
     * @arg: state_input_trajectory: 
     * 
     * In theory, raw_trajectory contain position and orientation of the wheelchair with respect to /odom frame
     */
    std::vector<std::pair<State, Action>> calculate_state_trajectory_one_gap(const std::deque<std::pair<nav_msgs::msg::Odometry, sensor_msgs::msg::Joy>>& raw_trajectory
                                            ,const Gap& gap);

    double cost_function(const State& state); 

    State state_transition_function(const State& state, const Action& action, double delta_t);
    
    double value_function(const State& state);

    double value_action_function(const State& state, const Action& action, double delta_t);

    /**
     * What is the probability that user perform an action given a state and a goal
     */
    double user_policy(const State& state, const Action& action, double delta_t);

private:
    //For value function
    std::vector<std::vector<double>> value_function_; //value_function_[theta_i][d_i]
    std::string value_function_filename_;

    //Synchronizing joystick and odom message 
    message_filters::Subscriber<sensor_msgs::msg::Joy> joy_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; //Sensor QoS should be use

    //Publisher 
    rclcpp::Publisher<wheelchair_control_support::msg::Gap>::SharedPtr gap_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_visualize_publisher_;

    // Saving joystick and odom message
    std::deque<std::pair<nav_msgs::msg::Odometry, sensor_msgs::msg::Joy>> raw_trajectory_;
    // state input trajectory to calculate intention
    sensor_msgs::msg::LaserScan scan_;

    //Finding gap
    std::vector<Gap> observed_gap_;
    FindGap gap_finder_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    /**Parameter**/
    unsigned int trajectory_length_; //How many past data point to keep (this is the maximum size of raw_trajectory_)
    WhillDynamic whill_dynamic_;
    double confident_threshold_; // Only gap with confident threshold 

    double delta_t_; //Time step
    double angle_weight_; //angle weight of cost function
    double distance_weight_; // distance weight of cost funcion
    double distance_threshold_; // distance > threshold will have the same contribution to the cost function
    double zeta_; 
    double discount_factor_; //gamma in MDP
    double max_distance_; //Maximum distance that was considered during the calculation of value function
    double number_of_distance_point_; //Number of distance value after discretization
    double max_abs_angle_distance_; 
    double number_of_angle_point_;



    /**
     * Helper function for convenient
     */
    /**
     * Return a transformation that transform a point from child_frame_id
     * to header.frame_id in Odometry message
     */
    geometry_msgs::msg::TransformStamped robot_to_odom_transform(const nav_msgs::msg::Odometry& odom_msg)
    {
        /**
         * The odometry message define a transformation from /odom frame
         * to /base_link(robot) frame. This means that this transformation
         * map point from /base_link frame to /odom frame
         */
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom_msg.header;
        transform.child_frame_id = odom_msg.child_frame_id;
        // Extract translation (position)
        transform.transform.translation.x = odom_msg.pose.pose.position.x;
        transform.transform.translation.y = odom_msg.pose.pose.position.y;
        transform.transform.translation.z = odom_msg.pose.pose.position.z;

        // Extract rotation (orientation in quaternion)
        transform.transform.rotation = odom_msg.pose.pose.orientation;
        return transform;   //Return position and orientation of base_link wrt odom
    }

    /**
     * Return a transformation that transform a point from header.frame_id
     * to child_frame_id in  Odometry message
     */
    geometry_msgs::msg::TransformStamped odom_to_robot_transform(const nav_msgs::msg::Odometry& odom_msg)
    {
        geometry_msgs::msg::TransformStamped inverse_transform;
        
        // Swap frame IDs (we want odom as the child frame of robot
        inverse_transform.header.frame_id = odom_msg.child_frame_id;
        inverse_transform.child_frame_id = odom_msg.header.frame_id;
        inverse_transform.header.stamp = odom_msg.header.stamp;

        // Convert geometry_msgs::Pose to tf2::Transform
        tf2::Transform tf_transform;
        tf2::fromMsg(odom_msg.pose.pose, tf_transform);

        // Compute the inverse transform
        tf2::Transform tf_inverse = tf_transform.inverse();

        // Convert back to geometry_msgs::TransformStamped
        inverse_transform.transform = tf2::toMsg(tf_inverse);

        return inverse_transform;
    }

    /**
     * Calculating the state (distance to gap, angular distance to gap) from 2 end point of gap
     * Gap is assumed to be in the reference frame of robot
     */
    State gap_endpoint_to_state(std::pair<double, double> left, std::pair<double, double> right)
    {
        State state;
        double mid_point_x{(left.first + right.first)/2};
        double mid_point_y{(left.second + right.second)/2};
        state.distance_to_goal_ = sqrt(mid_point_x*mid_point_x + mid_point_y*mid_point_y);
        state.angular_distance_to_goal_ = abs(atan2(mid_point_y, mid_point_x));
        return state;
    }

    Action joystick_input_to_action(const sensor_msgs::msg::Joy& joy_msg)
    {
        Action action;
        //Linear velocity
        /*Forward movement*/
        if (joy_msg.axes[1] >= 0)
        {
            action.angular_vel_ = joy_msg.axes[1] * whill_dynamic_.max_linear_vel_;
        }
        //backward movement
        else if (joy_msg.axes[1] < 0)
        {
            action.angular_vel_ = -joy_msg.axes[1] * whill_dynamic_.min_linear_vel_;
        }

        //Angular velocity
        action.angular_vel_ = joy_msg.axes[0] * whill_dynamic_.max_yaw_rate_;
        return action;
    }
};

#endif