#include "wheelchair_control_support/IntendedGoalEstimator.hpp"
#include "wheelchair_control_support/msg/gap.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <nlohmann/json.hpp>


GapIntentionEstimator::GapIntentionEstimator()
    : rclcpp::Node("gap_intention_estimator")
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("wheelchair_control_support");
    std::string value_function_filepath = package_share_directory + "/data/value_function_very_fined_changed_cost.json";
    std::string value_function_savepath = package_share_directory + "/data/check_value_function.csv";
    get_value_function(value_function_filepath, value_function_);
    // save_value_function(value_function_savepath, value_function_);

    /**
     * Set up parameter
     *  */ 
    trajectory_length_ = static_cast<int>(3/delta_t_); 
    confident_threshold_ = 0.5;
    //Whill dynamic need to be added

    //Prepare subscriber and check for subscribe frequency
    std::string joy_topic{"/whill/states/joy"};
    std::string odom_topic{"/whill/odom"};
    std::string scan_topic{"/filtered_scan"};
    setup_subscriber(joy_topic, odom_topic, scan_topic);

    //Initializing publisher to publish gap message
    gap_publisher_ = this->create_publisher<wheelchair_control_support::msg::Gap>("/intended_gap", 1);
    gap_visualize_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "gap_visualize", 1
        );

    //Initialize tf
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    

}

void GapIntentionEstimator::get_value_function(const std::string& value_function_filepath, GapIntentionEstimator::tensor3d& value_function)
{
    std::ifstream file(value_function_filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file: " + value_function_filepath);
    }

    // Parse JSON
    nlohmann::json j;
    file >> j;
    file.close();

    value_function.clear();  // Clear existing data

        try {
        // Extract metadata (throws an exception if any key is missing)
        nlohmann::json metadata = j.at("metadata");
        delta_t_ = metadata.at("delta_t");
        discount_factor_ = metadata.at("discount_factor");
        max_distance_ = metadata.at("max_distance");
        number_of_distance_point_ = metadata.at("number_of_distance_point");
        max_abs_angle_distance_ = metadata.at("max_abs_angle_distance");
        number_of_angle_point_ = metadata.at("number_of_angle_point");

        // Extract cost function parameters
        nlohmann::json cost_params = j.at("cost_function_param");
        w_d_ = cost_params.at("w_d");
        w_theta_= cost_params.at("w_theta");
        w_theta_g_ = cost_params.at("w_theta_g");
        k_theta_ = cost_params.at("k_theta");
        k_d_ = cost_params.at("k_d");
        k_theta_g_ = cost_params.at("k_theta_g");
        distance_threshold_ = cost_params.at("distance_threshold");

        // Clear existing matrix data
        value_function.clear();

        // Extract matrix values into 3D vector
        nlohmann::json matrix_data = j.at("matrix");
        for (const auto& distance : matrix_data) // For each distance value 
        {
            std::vector<std::vector<double>> second_dimension;
            for (const auto& theta : distance)
            {
                std::vector<double> third_dimesion;
                for (const auto& theta_g: theta)
                {
                    third_dimesion.push_back(theta_g.get<double>());
                }
                second_dimension.push_back(third_dimesion);
            }
            value_function.push_back(second_dimension);
        }

        std::cout << "Value function successfully loaded from: " << value_function_filepath << std::endl;
    }
    catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("JSON Parsing Error: " + std::string(e.what()));
    }
}

void GapIntentionEstimator::save_value_function(const std::string& output_filename, const GapIntentionEstimator::tensor3d& value_function)
{

    std::ofstream file(output_filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file for writing: " + output_filename);
    }

    // Write the value function matrix to CSV
    for (const auto& matrix : value_function) {  // Iterate over first dimension (slices)
        for (const auto& row : matrix) {  // Iterate over second dimension (rows)
            for (size_t i = 0; i < row.size(); ++i) {  // Iterate over third dimension (columns)
                file << row[i];
                if (i < row.size() - 1) {
                    file << ",";  // Add comma except for the last column
                }
            }
            file << "\n";  // New line for each row
        }
        file << "\n";  // Blank line to separate 2D slices
    }

    file.close();
    std::cout << "Value function successfully saved to: " << output_filename << std::endl;
}

void GapIntentionEstimator::setup_subscriber(const std::string& joy_topic, const std::string& odom_topic, const std::string& scan_topic)
{
    // rclcpp::QoS qos = rclcpp::SensorDataQoS();
    joy_sub_.subscribe(this, joy_topic);
    odom_sub_.subscribe(this, odom_topic);

    unsigned char queue_size{10};
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size), joy_sub_, odom_sub_);
    sync_->registerCallback(std::bind(&GapIntentionEstimator::joy_odom_callback, this, std::placeholders::_1, std::placeholders::_2));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS(),                                                                         
                                                                        std::bind(&GapIntentionEstimator::laser_scan_callback, this, std::placeholders::_1));
}

void GapIntentionEstimator::joy_odom_callback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    //Checking for subscription rate
    // wheelchair_control_support::msg::Gap found_gap;
    // found_gap.header = odom_msg->header;
    // found_gap.left_point = 0;
    // found_gap.middle_point = 0;
    // found_gap.right_point = 0;
    // gap_publisher_->publish(found_gap);

    std::cout << " joy odom update\n";
    //Save new pair of odom and joy message to a deque
    raw_trajectory_.push_front(std::make_pair(*odom_msg, *joy_msg));
    // Ensure the deque does not exceed `trajectory_length_`
    if (raw_trajectory_.size() > trajectory_length_) {
        raw_trajectory_.pop_back();  // Remove the oldest element
    }
}

void GapIntentionEstimator::laser_scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
{
    scan_ = *scan_msg;
    gap_finder_.find_gap_from_scan(*scan_msg, observed_gap_);
    gap_finder_.merge_gap(*scan_msg, observed_gap_);
    std::cout << "Observed Gaps Size: " << observed_gap_.size() << ", Trajectory Size: " << raw_trajectory_.size() << std::endl;
    if(!observed_gap_.empty() && !raw_trajectory_.empty()) //If gaps is found and have trajectory information
    {
        // Calculate gap logit
        // Using log-sum-exp trick for numerically stable calculation
        // Step 1: Compute log probability
        std::vector<double> gap_logits;
        double max_logit = -std::numeric_limits<double>::infinity();
        for(auto& gap : observed_gap_)
        {
            auto state_action_trajectory {calculate_state_trajectory_one_gap(raw_trajectory_, gap)};
            double gap_logit{0};
            for(const auto& state_action_pair : state_action_trajectory)
            {
                gap_logit+= log(user_policy(state_action_pair.first, state_action_pair.second, delta_t_));

            }
            
            // std::cout << "Cost at current: " << cost_function(state_action_trajectory[0].first) << "\n";
            // std::cout << "User policy before exponential: " << log(user_policy(state_action_trajectory[0].first, state_action_trajectory[0].second, delta_t_)) << "\n";
            std::cout << std::endl;
            gap_logits.push_back(gap_logit);
            max_logit = std::max(max_logit, gap_logit);
        }

        // Step 2: Normalized using Log-Sum-Exp
        double log_sum_exp = 0.0;
        for (double logit : gap_logits)
        {
            log_sum_exp += exp(logit - max_logit);
        }
        log_sum_exp = max_logit + log(log_sum_exp);

        // Step 3: Convert back to probabilities and set confidence
        for (std::size_t i = 0; i < observed_gap_.size(); ++i)
        {
            double prob = exp(gap_logits[i] - log_sum_exp);
            observed_gap_[i].set_confident(prob);
            std::cout << "Gap confident: " << prob << "\n";
        }
    }
    int id = 0;
    gap_visualize_publisher_->publish(gap_finder_.visualize_gaps(*scan_msg, observed_gap_, confident_threshold_,id));

    wheelchair_control_support::msg::Gap found_gap;
    if(!observed_gap_.empty())
    {
        auto max_gap = std::max_element(observed_gap_.begin(), observed_gap_.end(), [](const Gap& a, const Gap& b) {return a.get_confident() < b.get_confident();});
        if(max_gap->get_confident() > confident_threshold_)
        {
            found_gap.header = scan_msg->header;
            found_gap.confident = max_gap->get_confident();
            found_gap.left_point_x = max_gap->get_l_cartesian().first;
            found_gap.left_point_y = max_gap->get_l_cartesian().second;
            found_gap.right_point_x = max_gap->get_r_cartesian().first;
            found_gap.right_point_y = max_gap->get_r_cartesian().second;
            found_gap.middle_point_x = (found_gap.left_point_x + found_gap.right_point_x)/2;
            found_gap.middle_point_y = (found_gap.left_point_y + found_gap.right_point_y)/2;
        }
        else
        {
            found_gap.header = scan_msg->header;
            found_gap.confident = -1;
            found_gap.left_point_x = 0;
            found_gap.left_point_y = 0;
            found_gap.middle_point_x = 0;
            found_gap.middle_point_y = 0;
            found_gap.right_point_x = 0;
            found_gap.right_point_y = 0;
        }
    }
    else
    {
        found_gap.header = scan_msg->header;
        found_gap.confident = -1;
        found_gap.left_point_x = 0;
        found_gap.left_point_y = 0;
        found_gap.middle_point_x = 0;
        found_gap.middle_point_y = 0;
        found_gap.right_point_x = 0;
        found_gap.right_point_y = 0;0;
    }
    gap_publisher_->publish(found_gap);
    std::cout << "Finish 1 scan \n\n\n\n";

    // exit(0);
}

std::vector<std::pair<GapIntentionEstimator::State, GapIntentionEstimator::Action>> GapIntentionEstimator::calculate_state_trajectory_one_gap(const std::deque<std::pair<nav_msgs::msg::Odometry, sensor_msgs::msg::Joy>>& raw_trajectory
                                                                ,const Gap& gap
                                                                )
{
    /**Position of a gap is with respect to the current position of the wheelchair, which is the  first element of the raw_trajectory.
     * Position of gap need to be transformed to odom coordinate gap-> laser -> wheelchair -> odom
     */

    std::vector<std::pair<State, Action>> state_input_trajectory;

    /**
     * Gap coordinate tranformation
     */
    auto left_point{gap.get_l_cartesian()};
    auto right_point{gap.get_r_cartesian()};

    //Transform gap from scan frame to robot frame
    try{
        std::string robot_frame_{"base_link"};
        std::string lidar_frame_{scan_.header.frame_id};
        geometry_msgs::msg::TransformStamped lidar_to_robot_transform;
        lidar_to_robot_transform = tf_buffer_->lookupTransform(robot_frame_, lidar_frame_, tf2::TimePointZero);
        geometry_msgs::msg::Pose left_point_pose;   //Create pose message to apply transformation
        geometry_msgs::msg::Pose right_point_pose;  
        geometry_msgs::msg::Pose left_point_pose_odom;
        geometry_msgs::msg::Pose right_point_pose_odom;
        left_point_pose.position.x = left_point.first;
        left_point_pose.position.y = left_point.second;
        left_point_pose.position.z = 0.;
        right_point_pose.position.x = right_point.first;
        right_point_pose.position.y = right_point.second;
        right_point_pose.position.z = 0.;

        //Add state-action pair to state-input trajectory 
        tf2::doTransform(left_point_pose, left_point_pose,lidar_to_robot_transform); 
        tf2::doTransform(right_point_pose, right_point_pose,lidar_to_robot_transform);
        State state{gap_endpoint_to_state({left_point_pose.position.x, left_point_pose.position.y}, 
                        {right_point_pose.position.x, right_point_pose.position.y})};
        Action action{joystick_input_to_action(raw_trajectory[0].second)};
        state_input_trajectory.push_back({state, action});


        /*Transform gap from current robot frame to odom*/
        geometry_msgs::msg::TransformStamped current_robot_to_odom = robot_to_odom_transform(raw_trajectory[0].first);
        // Tranform gap to odom frame
        tf2::doTransform(left_point_pose, left_point_pose_odom,current_robot_to_odom);
        tf2::doTransform(right_point_pose, right_point_pose_odom,current_robot_to_odom);

        /* Transform gap from /Odom to past robot position*/
        for(std::size_t i = 1; i < raw_trajectory.size();i++)
        {
            auto past_pos{raw_trajectory[i].first};
            auto odom_to_past_pos_transform = odom_to_robot_transform(past_pos);
            tf2::doTransform(left_point_pose_odom, left_point_pose,odom_to_past_pos_transform); 
            tf2::doTransform(right_point_pose_odom, right_point_pose,odom_to_past_pos_transform);
            State state  = gap_endpoint_to_state({left_point_pose.position.x, left_point_pose.position.y}, 
                        {right_point_pose.position.x, right_point_pose.position.y});
            Action action = joystick_input_to_action(raw_trajectory[i].second);
            state_input_trajectory.push_back({state, action});


        }
    }catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return {};
    }

    // for (auto state: state_input_trajectory)
    // {
    //     std::cout << "State from calculate_state_trajectory_one_gap function: " << state.first.distance_to_goal_ << "\n";
    // }

    return state_input_trajectory;

}


double GapIntentionEstimator::cost_function(const GapIntentionEstimator::State& state)
{
    double d = std::min(state.distance_to_goal_, distance_threshold_);
    return w_d_*state.distance_to_goal_ + w_theta_*(1-exp(-k_theta_*d))*abs(state.angular_distance_to_goal_) + 
            w_theta_g_*exp(-k_theta_g_*d)*abs(state.orientation_of_goal_ + abs(state.orientation_of_goal_ - state.angular_distance_to_goal_));
}

GapIntentionEstimator::State GapIntentionEstimator::state_transition_function(const GapIntentionEstimator::State& state, const GapIntentionEstimator::Action& action, double delta_t)
{
    State new_state;

    double x_t = state.distance_to_goal_*cos(state.angular_distance_to_goal_);
    double y_t = state.distance_to_goal_*sin(state.angular_distance_to_goal_);

    // Apply transformation matrix
    double cos_wt = cos(action.angular_vel_ * delta_t);
    double sin_wt = sin(action.angular_vel_ * delta_t);
    
    double x_next = cos_wt * x_t + sin_wt * y_t - cos_wt * action.linear_vel_ * delta_t;
    double y_next = -sin_wt * x_t + cos_wt * y_t + sin_wt * action.linear_vel_ * delta_t;


    new_state.distance_to_goal_ = sqrt(x_next*x_next + y_next*y_next);
    new_state.angular_distance_to_goal_   = atan2(y_next, x_next);
    new_state.orientation_of_goal_ = state.orientation_of_goal_ - action.angular_vel_*delta_t;

    return new_state;
}

double GapIntentionEstimator::value_function(const GapIntentionEstimator::State& state)
{
    if(value_function_.empty() || value_function_[0].empty()){
        throw std::runtime_error("Value function is empty");
    }
    int distance_sample_size = number_of_distance_point_;
    int anglular_distance_sample_size = number_of_angle_point_;
    double max_distance = max_distance_;
    double max_angular_distance = max_abs_angle_distance_;
    double distance_step = (max_distance-0)/(number_of_distance_point_ - 1);
    double angle_step = (2*max_abs_angle_distance_)/(number_of_angle_point_-1);

    
    // unsigned int distance_index = (int) std::round(state.distance_to_goal_ /distance_step);
    // unsigned int angular_index = (int) std::round((state.angular_distance_to_goal_ - -max_abs_angle_distance_)/angle_step);
    // unsigned int orientation_index = (int)std::round((state.orientation_of_goal_ - max_abs_angle_distance_)/angle_step);

    // //Boundary checking for distance_index and angular_index is necessary

    // if(distance_index >= distance_sample_size) {distance_index = distance_sample_size-1;}
    // if(angular_index >= anglular_distance_sample_size){angular_index = anglular_distance_sample_size-1;}
    // if(orientation_index >= anglular_distance_sample_size){orientation_index = anglular_distance_sample_size-1;}

    // double ret = value_function_[distance_index][angular_index][orientation_index];

    /*-------------------------TRILINEAR INTERPOLATION------------------------------------*/
    double distance_idx_real = state.distance_to_goal_ / distance_step;
    double angular_idx_real = (state.angular_distance_to_goal_ + max_abs_angle_distance_) / angle_step;
    double orientation_idx_real = (state.orientation_of_goal_ + max_abs_angle_distance_) / angle_step;
    
    // Get the integer indices for the lower and upper bounds
    int d0 = std::floor(distance_idx_real); 
    int d1 = std::ceil(distance_idx_real);
    int a0 = std::floor(angular_idx_real);
    int a1 = std::ceil(angular_idx_real);
    int o0 = std::floor(orientation_idx_real);
    int o1 = std::ceil(orientation_idx_real);
    
    // Clamp indices to valid range 
    d0 = std::max(0, std::min(d0, distance_sample_size - 1)); // In case state.distance > distance , this is the last index
    d1 = std::max(0, std::min(d1, distance_sample_size - 1)); // In case state.distance > distance , this is the last index
    a0 = std::max(0, std::min(a0, anglular_distance_sample_size - 1));
    a1 = std::max(0, std::min(a1, anglular_distance_sample_size - 1));
    o0 = std::max(0, std::min(o0, anglular_distance_sample_size - 1));
    o1 = std::max(0, std::min(o1, anglular_distance_sample_size - 1));
    
    // Compute the fractional distances
    double d_weight = distance_idx_real - d0;
    double a_weight = angular_idx_real - a0;
    double o_weight = orientation_idx_real - o0;
    
    // Fetch the eight surrounding values
    double c000 = value_function_[d0][a0][o0];
    double c100 = value_function_[d1][a0][o0];
    double c010 = value_function_[d0][a1][o0];
    double c110 = value_function_[d1][a1][o0];
    double c001 = value_function_[d0][a0][o1];
    double c101 = value_function_[d1][a0][o1];
    double c011 = value_function_[d0][a1][o1];
    double c111 = value_function_[d1][a1][o1];
    
    // Perform trilinear interpolation
    double c00 = c000 * (1 - d_weight) + c100 * d_weight;
    double c01 = c001 * (1 - d_weight) + c101 * d_weight;
    double c10 = c010 * (1 - d_weight) + c110 * d_weight;
    double c11 = c011 * (1 - d_weight) + c111 * d_weight;
    
    double c0 = c00 * (1 - a_weight) + c10 * a_weight;
    double c1 = c01 * (1 - a_weight) + c11 * a_weight;
    
    double ret = c0 * (1 - o_weight) + c1 * o_weight;

    return ret;
}

double GapIntentionEstimator::value_action_function(const State& state, const Action& action, double delta_t)
{   
    return cost_function(state) + discount_factor_*value_function(state_transition_function(state, action, delta_t));
}

double GapIntentionEstimator::user_policy(const State& state, const Action& action, double delta_t)
{
    // std::cout << "V " << value_function(state) << std::endl;
    // std::cout << "Q " << value_function(state) << std::endl;
    return exp(0.02*(value_function(state) - value_action_function(state, action, delta_t)));
    //return exp(value_function(state));
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapIntentionEstimator>());
    rclcpp::shutdown();
    return 0;
}