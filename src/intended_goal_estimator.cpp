#include "wheelchair_control_support/IntendedGoalEstimator.hpp"
#include "wheelchair_control_support/msg/gap.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <nlohmann/json.hpp>


GapIntentionEstimator::GapIntentionEstimator()
    : rclcpp::Node("gap_intention_estimator")
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("wheelchair_control_support");
    std::string value_function_filepath = package_share_directory + "/data/value_function.json";
    std::string value_function_savepath = package_share_directory + "/data/check_value_function.csv";
    get_value_function(value_function_filepath, value_function_);
    // save_value_function(value_function_savepath, value_function_);

    /**
     * Set up parameter
     *  */ 
    trajectory_length_ = static_cast<int>(5/delta_t_); 
    confident_threshold_ = 0.8;
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

void GapIntentionEstimator::get_value_function(const std::string& value_function_filepath, std::vector<std::vector<double>>& value_function)
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
        angle_weight_ = cost_params.at("angle_weight");
        distance_weight_ = cost_params.at("distance_weight");
        distance_threshold_ = cost_params.at("distance_threshold");
        zeta_ = cost_params.at("zeta");

        // Clear existing matrix data
        value_function.clear();

        // Extract matrix values into 2D vector
        nlohmann::json matrix_data = j.at("matrix");
        for (const auto& row : matrix_data) // For each distance value 
        {
            std::vector<double> row_data; 
            for (const auto& val : row) { // For each angle distance value in each distance value
                row_data.push_back(val.get<double>());
            }
            value_function.push_back(row_data);
        }

        std::cout << "Value function successfully loaded from: " << value_function_filepath << std::endl;
    }
    catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("JSON Parsing Error: " + std::string(e.what()));
    }
}

void GapIntentionEstimator::save_value_function(const std::string& output_filename, const std::vector<std::vector<double>>& value_function)
{

    std::ofstream file(output_filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file for writing: " + output_filename);
    }

    // Write the value function matrix to CSV
    for (const auto& row : value_function) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";  // Add comma except for last column
            }
        }
        file << "\n";  // New line for each row
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

    //Save new pair of odom and joy message to a deque
    raw_trajectory_.push_front(std::make_pair(*odom_msg, *joy_msg));
    // Ensure the deque does not exceed `trajectory_length_`
    if (raw_trajectory_.size() > trajectory_length_) {
        raw_trajectory_.pop_back();  // Remove the oldest element
    }
}

void GapIntentionEstimator::laser_scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
{
    gap_finder_.find_gap_from_scan(*scan_msg, observed_gap_);
    gap_finder_.merge_gap(*scan_msg, observed_gap_);
    std::cout << "Observed Gaps Size: " << observed_gap_.size() << ", Trajectory Size: " << raw_trajectory_.size() << std::endl;
    if(!observed_gap_.empty() && !raw_trajectory_.empty()) //If gaps is found and have trajectory information
    {
        // Calculate gap logit
        std::cout << "Inside main if \n";
        double logit_sum{0};
        for(auto& gap : observed_gap_)
        {
            auto state_action_trajectory {calculate_state_trajectory_one_gap(raw_trajectory_, gap)};
            //Calculate the logit for this gap
            double gap_logit{1};
            for(const auto& state_action_pair : state_action_trajectory)
            {
                gap_logit*= user_policy(state_action_pair.first, state_action_pair.second, delta_t_);
            }
            gap.set_confident(gap_logit);
            logit_sum += gap_logit;
        }

        //Normalize gap logit
        for(auto& gap: observed_gap_)
        {
            gap.set_confident(gap.get_confident()/logit_sum);
            std::cout <<  "Gap condfident " <<gap.get_confident() << std::endl;
        }
    }
    int id = 0;
    gap_visualize_publisher_->publish(gap_finder_.visualize_gaps(*scan_msg, observed_gap_, id));
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
        std::string lidar_frame_{"base_link"};
        geometry_msgs::msg::TransformStamped lidar_to_robot_transform;
        lidar_to_robot_transform = tf_buffer_->lookupTransform(robot_frame_, lidar_frame_, tf2::TimePointZero);
        geometry_msgs::msg::Pose left_point_pose;   //Create pose message to apply transformation
        geometry_msgs::msg::Pose right_point_pose;  
        left_point_pose.position.x = left_point.first;
        left_point_pose.position.y = left_point.second;
        left_point_pose.position.z = 0.;
        right_point_pose.position.x = right_point.first;
        right_point_pose.position.y = right_point.second;
        right_point_pose.position.z = 0.;
        tf2::doTransform(left_point_pose, left_point_pose,lidar_to_robot_transform); //Apply transformation to current robot frame
        tf2::doTransform(right_point_pose, right_point_pose,lidar_to_robot_transform);
        //Add state-action pair to state-input trajectory 
        State state{gap_endpoint_to_state({left_point_pose.position.x, left_point_pose.position.y}, 
                        {right_point_pose.position.x, right_point_pose.position.y})};
        Action action{joystick_input_to_action(raw_trajectory[0].second)};
        state_input_trajectory.push_back({state, action});

        /*Transform gap from current robot frame to odom*/
        // Convert Odometry to TransformStamped (Odom -> Base_link)
        geometry_msgs::msg::TransformStamped current_robot_to_odom = robot_to_odom_transform(raw_trajectory[0].first);
        // Tranform gap to odom frame
        tf2::doTransform(left_point_pose, left_point_pose,current_robot_to_odom);
        tf2::doTransform(right_point_pose, right_point_pose,current_robot_to_odom);
    
        /* Transform gap from /Odom to past robot position*/
        for(std::size_t i = 1; i < raw_trajectory.size();i++)
        {
            auto past_pos{raw_trajectory[i].first};
            auto odom_to_past_pos_transform = odom_to_robot_transform(past_pos);
            tf2::doTransform(left_point_pose, left_point_pose,odom_to_past_pos_transform);
            tf2::doTransform(right_point_pose, right_point_pose,odom_to_past_pos_transform);
            State state {gap_endpoint_to_state({left_point_pose.position.x, left_point_pose.position.y}, 
                        {right_point_pose.position.x, right_point_pose.position.y})};
            Action action {joystick_input_to_action(raw_trajectory[i].second)};
            state_input_trajectory.push_back({state, action});
        }

    }catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return {};
    }


    return state_input_trajectory;

}


double GapIntentionEstimator::cost_function(const GapIntentionEstimator::State& state)
{
    double C_d{0};
    if (state.distance_to_goal_ > distance_threshold_) {C_d = zeta_;}
    else if(state.distance_to_goal_<distance_threshold_){C_d = zeta_*state.distance_to_goal_ / distance_threshold_;}
    return angle_weight_*state.angular_distance_to_goal_ + distance_weight_*C_d;
}

GapIntentionEstimator::State GapIntentionEstimator::state_transition_function(const GapIntentionEstimator::State& state, const GapIntentionEstimator::Action& action, double delta_t)
{
    State new_state;

    // Compute intermediate terms

    // Ensure non-negative argument for sqrt()
    new_state.distance_to_goal_ = std::sqrt(std::max(state.distance_to_goal_*state.distance_to_goal_ 
                                                    - 2*state.distance_to_goal_*action.linear_vel_*cos(state.angular_distance_to_goal_)*delta_t 
                                                    + action.linear_vel_*action.linear_vel_*delta_t*delta_t, 0.0));

    // Update angular distance with wrap around at boundary [-pi, pi)
    new_state.angular_distance_to_goal_ = std::atan2(
                                            std::sin(state.angular_distance_to_goal_ - action.angular_vel_ * delta_t),
                                            std::cos(state.angular_distance_to_goal_ - action.angular_vel_ * delta_t));

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

    int distance_index = static_cast<int>((state.distance_to_goal_/max_distance)*distance_sample_size);
    int angular_index = static_cast<int>((state.angular_distance_to_goal_/max_angular_distance)*anglular_distance_sample_size);

    //Boundary checking for distance_index and angular_index is necessary
    if(distance_index >= distance_sample_size) {distance_index = distance_sample_size-1;}
    if(angular_index >= anglular_distance_sample_size){angular_index = anglular_distance_sample_size-1;}

    double ret = value_function_[distance_index][angular_index];
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
    return exp(value_function(state) - value_action_function(state, action, delta_t));
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapIntentionEstimator>());
    rclcpp::shutdown();
    return 0;
}