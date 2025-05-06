#include <iostream>
#include <random>
#include "wheelchair_control_support/ShareControl.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wheelchair_control_support/CircularFootprint.hpp"
#include "wheelchair_control_support/RectangularFootprint.hpp"

using namespace std::chrono_literals;

ShareControl::ShareControl()
    : rclcpp::Node("share_control_node")
{
    std::cout << "Instantiate share control node" << std::endl;

    /*Initializing parameters*/
    this->declare_parameter("predict_time", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("predict_timestep", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("scan_to_obstacle_range", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("skip_point", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("max_linear_vel", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("min_linear_vel", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_yaw_rate", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_acceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_decceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_yaw_acceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("period", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("linear_velocity_sample_size", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("yaw_rate_sample_size", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("rectangle_footprint.use", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("rectangle_footprint.height", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("rectangle_footprint.width", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("footprint_radius", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("joystick_noise", rclcpp::PARAMETER_BOOL);

    predict_time_           = this->get_parameter("predict_time").as_double();
    predict_timestep_       = this->get_parameter("predict_timestep").as_double();
    scan_to_obs_range_      = this->get_parameter("scan_to_obstacle_range").as_double();
    skip_point_             = this->get_parameter("skip_point").as_int();
    period_                 = this->get_parameter("period").as_double();
    linear_vel_sample_size_ = this->get_parameter("linear_velocity_sample_size").as_int();
    yaw_rate_sample_size_   = this->get_parameter("yaw_rate_sample_size").as_int();
    joystick_noise_         = this->get_parameter("joystick_noise").as_bool();
    whill_dynamic_.max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    whill_dynamic_.min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
    whill_dynamic_.max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    whill_dynamic_.max_yaw_acceleration_ = this->get_parameter("max_yaw_acceleration").as_double();
    whill_dynamic_.max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    whill_dynamic_.max_deceleration_ = this->get_parameter("max_decceleration").as_double();
    {
        bool use_rectangle_footprint =this->get_parameter("rectangle_footprint.use").as_bool();
        double rectangle_footprint_height = this->get_parameter("rectangle_footprint.height").as_double();
        double rectangle_footprint_width = this->get_parameter("rectangle_footprint.width").as_double();
        double circle_footprint_radius = this->get_parameter("footprint_radius").as_double();
        if (use_rectangle_footprint)
        {
            footprint_ptr_ = std::make_unique<RectangularFootprint>(rectangle_footprint_width,rectangle_footprint_height);  //Width = 0.7 and length = 1. Because the x-direction is forward
        }
        else
        {
            footprint_ptr_ = std::make_unique<CircularFootprint>(circle_footprint_radius);
        }
    }
    /*Initializing tf listener*/
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /*Initializing subscriber and publisher*/
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/filtered_scan", rclcpp::SensorDataQoS(),                                                                         
                                                                        std::bind(&ShareControl::laser_scan_callback, this, std::placeholders::_1));
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/whill/odom", rclcpp::SensorDataQoS(),
                                                                          std::bind(&ShareControl::odom_callback, this, std::placeholders::_1));
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/whill/states/joy", rclcpp::SensorDataQoS(),
                                                                       std::bind(&ShareControl::joystick_callback, this, std::placeholders::_1));
    gap_subscriber_ = this->create_subscription<wheelchair_control_support::msg::Gap>("/intended_gap", 1,
                                                                                      std::bind(&ShareControl::gap_callback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/whill/controller/cmd_vel", 1);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/controller/joy", 1);
    obs_list_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_list", 10);
    traj_visualizer_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_visualization", 10);
    user_traj_visualizer_pub = this->create_publisher<visualization_msgs::msg::Marker>("/user_trajectory_visualization", 1);

    /*Run main_process in loop*/
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_), std::bind(&ShareControl::main_process, this));
}

ShareControl::State::State(const double x, const double y, const double yaw, const double velocity, const double yaw_rate)
    : x_{x}, y_{y}, yaw_{yaw}, velocity_{velocity}, yaw_rate_{yaw_rate}
{
}

void ShareControl::laser_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
    if (!is_scan_updated_)
    {
        scan_to_obstacle(msg);
        is_scan_updated_ = true;
    }
}

void ShareControl::odom_callback(const nav_msgs::msg::Odometry &msg)
{
    if (!is_odom_updated_)
    {
        odom_ = msg;
        is_odom_updated_ = true;
    }
}

void ShareControl::joystick_callback(const sensor_msgs::msg::Joy &msg)
{
    if (!is_joystick_updated_)
    {
        joystick_ = msg;
        is_joystick_updated_ = true;
    }
}

void ShareControl::gap_callback(const wheelchair_control_support::msg::Gap &msg)
{
    observed_gap_ = msg;
    is_gap_updated_ = true;
}   

void ShareControl::scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan)
{
    std::string lidar_frame{scan.header.frame_id};
    float angle{scan.angle_min};

    obs_list_.header.frame_id = robot_frame_;
    obs_list_.header.stamp = scan.header.stamp;
    obs_list_.poses.clear();
    for (std::size_t i = 0; i < scan.ranges.size(); i += skip_point_)
    {
        float r = scan.ranges[i];
        /*Ignore r out of scan range*/
        if (!std::isfinite(r) || r > scan_to_obs_range_)
        {
            angle += scan.angle_increment * skip_point_;
            continue;
        }
        geometry_msgs::msg::Pose obstacle_pose;
        obstacle_pose.position.x = r * cos(angle);
        obstacle_pose.position.y = r * sin(angle);
        angle += scan.angle_increment * skip_point_;
        /**Tranform the pose from lidar coordinate to robot coordinate (/base_footprint)**/
        if (lidar_frame!=robot_frame_){
            try 
            {
                geometry_msgs::msg::TransformStamped to_robot_frame = tf_buffer_->lookupTransform(
                    robot_frame_, lidar_frame,
                    tf2::TimePointZero);
                tf2::doTransform(obstacle_pose, obstacle_pose, to_robot_frame);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                            robot_frame_.c_str(), lidar_frame.c_str(), ex.what());
                return;
            }
        }
        // Lidar reading inside footprint of robot at the current time is not counted as obstacle 
        // if(!footprint_ptr_->check_point_inside_footprint(obstacle_pose.position.x, obstacle_pose.position.y))
        // {
            obs_list_.poses.push_back(obstacle_pose);
        // }
    }
    obs_list_pub_->publish(obs_list_);
}

geometry_msgs::msg::Twist ShareControl::calculate_velocity_from_joy()
{
    geometry_msgs::msg::Twist joy_vel{};
    if (joystick_noise_)
    {
        /*Add noise to joystick*/
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0, 0.4);
        joystick_.axes[0] += noise(gen);
        joystick_.axes[1] += noise(gen);

        /*Limit the joystick value*/
        if (joystick_.axes[0] > 1)
        {
            joystick_.axes[0] = 1;
        }
        else if (joystick_.axes[0] < -1)
        {
            joystick_.axes[0] = -1;
        }
        if (joystick_.axes[1] > 1)
        {
            joystick_.axes[1] = 1;
        }
        else if (joystick_.axes[1] < -1)
        {
            joystick_.axes[1] = -1;
        }
    }
    
    if (is_joystick_updated_)
    {

        //Linear velocity
        /*Forward movement*/
        if (joystick_.axes[1] >= 0)
        {
            joy_vel.linear.x = joystick_.axes[1] * whill_dynamic_.max_linear_vel_;
        }
        /*Backward movement*/
        else if (joystick_.axes[1] < 0)
        {
            joy_vel.linear.x = -joystick_.axes[1] * whill_dynamic_.min_linear_vel_;
        }

        //Angular velocity
        joy_vel.angular.z = joystick_.axes[0] * whill_dynamic_.max_yaw_rate_;
    }
    // std::cout << "Joy vel linear: " << joy_vel.linear.x << std::endl;
    // std::cout << "Joy vel angular: " << joy_vel.angular.z << std::endl;
    return joy_vel;
}

sensor_msgs::msg::Joy ShareControl::calculate_joy_from_velocity(geometry_msgs::msg::Twist  vel)
{
    sensor_msgs::msg::Joy joy{};
    joy.axes.resize(2);
    /*Forward backward movement*/
    if (vel.linear.x >= 0 && vel.linear.x <= whill_dynamic_.max_linear_vel_)
    {
        joy.axes[1] = vel.linear.x / whill_dynamic_.max_linear_vel_ ;
    }
    else if (vel.linear.x < 0 && vel.linear.x >= whill_dynamic_.min_linear_vel_)
    {
        joy.axes[1] = - vel.linear.x/whill_dynamic_.min_linear_vel_ ;
    }

    if(vel.angular.z >= - whill_dynamic_.max_yaw_rate_ && vel.angular.z <= whill_dynamic_.max_yaw_rate_)
    {
        joy.axes[0] =vel.angular.z/whill_dynamic_.max_yaw_rate_;
        }
    return joy;
}

std::vector<ShareControl::State> ShareControl::generate_trajectory(const double linear_vel, const double yaw_rate)
{
    int sample_size = static_cast<int>(predict_time_ / predict_timestep_);
    std::vector<State> trajectory;
    trajectory.reserve(sample_size);
    State state(0., 0., 0., linear_vel, yaw_rate);
    for (int i = 0; i < sample_size; ++i)
    {   
        State  next_state = state.next_state(predict_timestep_, 0.425, 0);
        trajectory.push_back(next_state);
        state = next_state;
    }
    return trajectory;
}

double ShareControl::calculate_vel_pair_cost(const double linear_vel, 
    const double yaw_rate, 
    const wheelchair_control_support::msg::Gap &gap,
    const double linear_vel_user,
    const double yaw_rate_user,
    const std::vector<State>& traj)
{
    const double w_user = 1.0;
    const double w_angle = (gap.confident>0)?0.3:0.0; //0.3
    const double w_distance = w_angle;
    const double w_obstacle = 0.1;
    const double k = 5;

    State current_state{0,0,0,linear_vel, yaw_rate};
    State next_state = traj.back();
    double user_cost = hypot(linear_vel - linear_vel_user, yaw_rate - yaw_rate_user);
    double obstacle_cost = exp(-closest_distance_to_obstacle(next_state));
    double angle_diff = next_state.yaw_ - std::atan2(gap.middle_point_y, gap.middle_point_x);
    if (abs(angle_diff) > M_PI)
    {
        angle_diff = 2 * M_PI - abs(angle_diff);
    }
    footprint_ptr_->move_footprint(next_state.x_, next_state.y_, next_state.yaw_);
    // double distance_diff = - 1 /footprint_ptr_->distance_from_point_to_footprint(gap.middle_point_x, gap.middle_point_y);

    return w_user * user_cost + 
            w_angle * abs(angle_diff)*exp(k*gap.confident);
            // w_distance *exp(k*gap.confident)* distance_diff  + 
            w_obstacle * obstacle_cost;
}


// void ShareControl::motion(ShareControl::State &state, double linear_vel, double yaw_rate)
// {
//     state.yaw_ += predict_timestep_*yaw_rate;
//     state.x_ += linear_vel*std::cos(state.yaw_)*predict_timestep_;
//     state.y_ += linear_vel*std::sin(state.yaw_)*predict_timestep_;
//     state.velocity_ =  linear_vel;
//     state.yawrate_ = yaw_rate;
// }

bool ShareControl::check_for_colllision(const std::vector<State> &traj)
{
    for (std::size_t i = 0; i <= traj.size(); i++)
    {
        State state = traj[i];
        footprint_ptr_->move_footprint(state.x_, state.y_, state.yaw_);
        for (geometry_msgs::msg::Pose obs : obs_list_.poses)
        {
            if (!footprint_ptr_->check_point_inside_footprint(obs.position.x, obs.position.y))
                continue;
            else
                return true;
        }
    }
    return false;
}

double ShareControl::closest_distance_to_obstacle(const State& state)
{
    double min_distance = 1000000;
    footprint_ptr_->move_footprint(state.x_, state.y_, state.yaw_);
    for (geometry_msgs::msg::Pose obs : obs_list_.poses)
    {
        double distance = footprint_ptr_->distance_from_point_to_footprint(obs.position.x, obs.position.y);
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    return min_distance;
}

double ShareControl::closest_distance_to_obstacle(const std::vector<State>& traj)
{
    double min_distance = 1000000;
    for (State state : traj)
    {
        double distance = closest_distance_to_obstacle(state);
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    return min_distance;
}

void ShareControl::trajectory_visualization(const std::vector<State> &traj)
{
    /*Delete all existing marker in Rviz*/
    visualization_msgs::msg::MarkerArray delete_marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = robot_frame_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker_array.markers.push_back(delete_marker);
    traj_visualizer_pub_->publish(delete_marker_array);

    /*Publish and visualize new marker*/
    visualization_msgs::msg::MarkerArray traj_with_footprint;
    int marker_id{0};
    for (auto state : traj)
    {
        footprint_ptr_->move_footprint(state.x_, state.y_, state.yaw_);
        traj_with_footprint.markers.push_back(footprint_ptr_->generate_footprint_marker(robot_frame_, marker_id));
        ++marker_id;
    }
    traj_visualizer_pub_->publish(traj_with_footprint);

}

void ShareControl::user_trajectory_visualization(const std::vector<State> &traj)
{
    visualization_msgs::msg::Marker traj_marker;
    traj_marker.header.frame_id = robot_frame_;
    traj_marker.header.stamp = joystick_.header.stamp; //Stamp same as input joystick
    traj_marker.ns = "user_desired_traj";
    traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::msg::Marker::ADD;
    traj_marker.id = 1000;
    traj_marker.scale.x = 0.05;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;
    for (auto state : traj)
    {
        geometry_msgs::msg::Point p;
        p.x = state.x_;
        p.y = state.y_;
        traj_marker.points.push_back(p);
    }
    user_traj_visualizer_pub->publish(traj_marker);
}

ShareControl::Window ShareControl::cal_dynamic_window()
{
    Window window;
    window.min_velocity_ = std::max(odom_.twist.twist.linear.x - whill_dynamic_.max_deceleration_ * period_, whill_dynamic_.min_linear_vel_);
    window.max_velocity_ = std::min(odom_.twist.twist.linear.x + whill_dynamic_.max_acceleration_ * period_, whill_dynamic_.max_linear_vel_);
    window.max_yaw_rate_ = std::min(odom_.twist.twist.angular.z + whill_dynamic_.max_yaw_acceleration_ * period_, whill_dynamic_.max_yaw_rate_);
    window.min_yaw_rate_ = std::max(odom_.twist.twist.angular.z - whill_dynamic_.max_yaw_acceleration_ * period_, -whill_dynamic_.max_yaw_rate_);
    return window;
}

std::vector<std::pair<double, double>> ShareControl::discretize_dynamic_window(const Window &window)
{
    double dv = (window.max_velocity_ - window.min_velocity_) / (linear_vel_sample_size_ - 1);
    double dw = (window.max_yaw_rate_ - window.min_yaw_rate_) / (yaw_rate_sample_size_ - 1);
    std::vector<std::pair<double, double>> vel_pair_list;  //pair of linear and angular velocity
    vel_pair_list.reserve(linear_vel_sample_size_*(yaw_rate_sample_size_+1)); 
    for (int i = 0;i<linear_vel_sample_size_;++i)
    {
        double linear_vel{window.min_velocity_ + i*dv};
        for (int j = 0; j<yaw_rate_sample_size_;++j)
        {
            double yaw_rate{window.min_yaw_rate_ + j*dw};
            vel_pair_list.push_back(std::make_pair(linear_vel, yaw_rate));
        }

        //Consider straight trajectory
        //vel_pair_list.push_back(std::make_pair(linear_vel,0.));
    }
    return vel_pair_list;
}

void ShareControl::reset_command_vel()
{
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
}

void ShareControl::main_process()
{
    if (is_scan_updated_ && is_odom_updated_ && is_joystick_updated_)
    {
        reset_command_vel();
        geometry_msgs::msg::Twist joy_vel_ = calculate_velocity_from_joy();
        std::vector<State> joy_traj = generate_trajectory(joy_vel_.linear.x, joy_vel_.angular.z);
        user_trajectory_visualization(joy_traj);

        if(abs(joy_vel_.linear.x) <= 1e-4 && abs(joy_vel_.angular.z) <= 1e-4) //Stop safely
        {
            cmd_vel_.linear.x = 0;
            cmd_vel_.angular.z = 0;   
        }
        else 
        {
            // if (observed_gap_.confident > 0){std::cout << "Intended gap detected" << std::endl;} else{std::cout << "No gap detected" << std::endl;}
            Window window = cal_dynamic_window();
            std::vector<std::pair<double, double>> vel_pair_list = discretize_dynamic_window(window);
            double min_cost{1e6};
            std::vector<State> best_traj;
            bool is_vel_updated{false};
            for (auto vel_pair : vel_pair_list)
            {
                std::vector<State> traj = generate_trajectory(vel_pair.first, vel_pair.second);
                if(check_for_colllision(traj))
                    {continue;}
                double cost = calculate_vel_pair_cost(vel_pair.first, vel_pair.second, observed_gap_, joy_vel_.linear.x, joy_vel_.angular.z, traj);
                if(cost < min_cost)
                {
                    best_traj = traj;
                    min_cost = cost;
                    cmd_vel_.linear.x = vel_pair.first;
                    cmd_vel_.angular.z = vel_pair.second;
                }
                is_vel_updated = true;
            }
            // std::cout << "Selected velocity pair: " << cmd_vel_.linear.x << " " << cmd_vel_.angular.z << std::endl;


            // // Enter recovery motion if no feasible trajectory found
            if(!is_vel_updated)
            {
                std::cout << "No feasible trajectory found \n Recovery mode" << std::endl;
                // cmd_vel_.linear.x = joy_vel_.linear.x;
                // cmd_vel_.angular.z = joy_vel_.angular.z;

                // double furthest_distance = -1e6;
                // for (auto vel_pair : vel_pair_list)
                // {
                //     State current_state{0,0,0,vel_pair.first, vel_pair.second};
                //     State next_state = current_state.next_state(period_);
                //     footprint_ptr_->move_footprint(next_state.x_, next_state.y_, next_state.yaw_);
                //     bool is_valid_velocity = true;  // Flag to check if the velocity is valid

                //     // Check if any obstacle is inside the footprint
                //     for (auto obs : obs_list_.poses)
                //     {
                //         if (footprint_ptr_->check_point_inside_footprint(obs.position.x, obs.position.y))
                //         {
                //             is_valid_velocity = false;  // If an obstacle is inside the footprint, invalidate the velocity
                //             break;  // No need to check further, break out of the loop
                //         }
                //     }

                //     // If no obstacle is inside the footprint, select this velocity
                //     if (is_valid_velocity)
                //     {
                //         // Select this velocity (i.e., the valid velocity pair)
                //         // Add your code here to process or save the valid velocity, for example:
                //         cmd_vel_.linear.x = vel_pair.first;
                //         cmd_vel_.angular.z = vel_pair.second;
                //         break;  // Exit the loop once a valid velocity is found
                //     }
                // }
                std::cout << "Selected velocity pair: " << cmd_vel_.linear.x << " " << cmd_vel_.angular.z << std::endl;
                // std::cout << "Furthest distance: " << furthest_distance << std::endl;
            }
            
            trajectory_visualization(best_traj);
            // std::cout << "Intended gap velocity done" << std::endl;
        }

        //Control with virtual joystick control
        joy_pub_->publish(calculate_joy_from_velocity(cmd_vel_));
        // vel_pub_->publish(cmd_vel_);
        is_scan_updated_ = false;
        is_odom_updated_ = false;
        is_joystick_updated_ = false;
    }
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShareControl>());
    rclcpp::shutdown();
    return 0;
}
