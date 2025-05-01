#include <random>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"  
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "wheelchair_control_support/shared_controller_node.hpp"

SharedControllerNode::SharedControllerNode() : rclcpp::Node("shared_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing shared controller node with MPPI controller");


    this->declare_parameter("joystick_noise", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("max_linear_vel", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("min_linear_vel", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_yaw_rate", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_acceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_decceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_yaw_acceleration", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("controller_frequency", rclcpp::PARAMETER_DOUBLE);


    joystick_noise_    = this->get_parameter("joystick_noise").as_bool();
    period_           = 1/this->get_parameter("controller_frequency").as_double();
    whill_dynamic_.max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    whill_dynamic_.min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
    whill_dynamic_.max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    whill_dynamic_.max_yaw_acceleration_ = this->get_parameter("max_yaw_acceleration").as_double();
    whill_dynamic_.max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    whill_dynamic_.max_deceleration_ = this->get_parameter("max_decceleration").as_double();

    // Initializing costmap and controller
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", std::string{get_namespace()}, "local_costmap");
    mppi_controller_ = std::make_shared<nav2_shared_mppi_controller::MPPISharedController>();
    smac_planner_ = std::make_shared<wheelchair_smac_planner::SmacPlannerHybrid>();

    //Initializing subscriber and publisher
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/whill/odom", rclcpp::SensorDataQoS(),
                                                    std::bind(&SharedControllerNode::odom_callback, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/whill/states/joy", rclcpp::SensorDataQoS(),
                                                std::bind(&SharedControllerNode::joystick_callback, this, std::placeholders::_1));
    gap_sub_ = this->create_subscription<wheelchair_control_support::msg::Gap>("/intended_gap", 1,
                                                    std::bind(&SharedControllerNode::gap_callback, this, std::placeholders::_1));
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/controller/joy", 1);
    traj_visualizer_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/user_trajectory_visualization", 1);
    plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_), std::bind(&SharedControllerNode::main_process, this));
}

void SharedControllerNode::configure()
{
    std::weak_ptr<rclcpp::Node> node = shared_from_this();

    // Configure the costmap
    costmap_ros_->configure();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);


    //Configure the controller
    mppi_controller_->configure(node, "shared_mppi_controller",costmap_ros_->getTfBuffer(), costmap_ros_);
    smac_planner_->configure(node, "smac_planner", costmap_ros_->getTfBuffer(), costmap_ros_);
}

void SharedControllerNode::activate()
{
    // Activate the costmap
    costmap_ros_->activate();
    mppi_controller_->activate();
    smac_planner_->activate();
}

SharedControllerNode::~SharedControllerNode()
{
    costmap_thread_.reset();

    // Deactivate and destroy the costmap
    costmap_ros_->stop();  // optional but safe if implemented
    costmap_ros_->deactivate();
    costmap_ros_->cleanup();

    mppi_controller_->deactivate();
    mppi_controller_->cleanup();

    smac_planner_->deactivate();
    smac_planner_->cleanup();
}


void SharedControllerNode::odom_callback(const nav_msgs::msg::Odometry &msg)
{
    if (!is_odom_updated_)
    {
        odom_ = msg;
        is_odom_updated_ = true;
    }
}

void SharedControllerNode::joystick_callback(const sensor_msgs::msg::Joy &msg)
{
    if (!is_joystick_updated_)
    {
        joystick_ = msg;
        is_joystick_updated_ = true;
    }
}

void SharedControllerNode::gap_callback(const wheelchair_control_support::msg::Gap &msg)
{
    observed_gap_ = msg;
    is_gap_updated_ = true;
}   

geometry_msgs::msg::Twist   SharedControllerNode::calculate_velocity_from_joy(sensor_msgs::msg::Joy & joy)
{
    geometry_msgs::msg::Twist joy_vel{};
    if (joystick_noise_)
    {
        /*Add noise to joystick*/
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0, 0.4);
        joy.axes[0] += noise(gen);
        joy.axes[1] += noise(gen);

        /*Limit the joystick value*/
        if (joy.axes[0] > 1)
        {
            joy.axes[0] = 1;
        }
        else if (joy.axes[0] < -1)
        {
            joy.axes[0] = -1;
        }
        if (joy.axes[1] > 1)
        {
            joy.axes[1] = 1;
        }
        else if (joy.axes[1] < -1)
        {
            joy.axes[1] = -1;
        }
    }
    
    if (is_joystick_updated_)
    {

        //Linear velocity
        /*Forward movement*/
        if (joy.axes[1] >= 0)
        {
            joy_vel.linear.x = joy.axes[1] * whill_dynamic_.max_linear_vel_;
        }
        /*Backward movement*/
        else if (joy.axes[1] < 0)
        {
            joy_vel.linear.x = -joy.axes[1] * whill_dynamic_.min_linear_vel_;
        }

        //Angular velocity
        joy_vel.angular.z = joy.axes[0] * whill_dynamic_.max_yaw_rate_;
    }
    // std::cout << "Joy vel linear: " << joy_vel.linear.x << std::endl;
    // std::cout << "Joy vel angular: " << joy_vel.angular.z << std::endl;
    return joy_vel;
}

sensor_msgs::msg::Joy   SharedControllerNode::calculate_joy_from_velocity(const geometry_msgs::msg::Twist & vel){
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

geometry_msgs::msg::PoseStamped SharedControllerNode::convert_gap_to_pose(const wheelchair_control_support::msg::Gap & gap)
{
    geometry_msgs::msg::PoseStamped gap_pose{};
    gap_pose.header.frame_id = "base_footprint";
    gap_pose.header.stamp = joystick_.header.stamp; //Stamp same as input joystick

    gap_pose.pose.position.z = 0.0;
    gap_pose.pose.position.x = gap.middle_point_x;
    gap_pose.pose.position.y = gap.middle_point_y;

    double angle = std::atan2(gap.left_point_x - gap.right_point_x, gap.right_point_y - gap.left_point_y);
    gap_pose.pose.orientation =  nav2_util::geometry_utils::orientationAroundZAxis(angle);

    return gap_pose;    
}

bool SharedControllerNode::validatePath(const nav_msgs::msg::Path &path)
{
    if(path.poses.size() == 0)
    {
        // RCLCPP_ERROR(get_logger(), "Invalide path");
        return false;
    }
    else
    {
        // RCLCPP_ERROR(get_logger(), "Valid path");
        return true;
    }
}

void SharedControllerNode::computePathToPose(const geometry_msgs::msg::PoseStamped &goal, nav_msgs::msg::Path& path)
{
    path.header.frame_id = "base_footprint";
    path.header.stamp = joystick_.header.stamp; //Stamp same as input joystick

    geometry_msgs::msg::PoseStamped start;
    costmap_ros_->getRobotPose(start);

    path = smac_planner_->createPlan(start, goal);
}


void SharedControllerNode::user_trajectory_visualization()
{
    auto user_vel = calculate_velocity_from_joy(joystick_);
    visualization_msgs::msg::Marker traj_marker;
    traj_marker.header.frame_id = "base_footprint";
    traj_marker.header.stamp = joystick_.header.stamp; //Stamp same as input joystick
    traj_marker.ns = "user_desired_traj";
    traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::msg::Marker::ADD;
    traj_marker.id = 1000;
    traj_marker.scale.x = 0.05;
    traj_marker.color.r = 0.8;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;

    //Calculate trajectory assuming constant velocity 
    geometry_msgs::msg::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    double yaw{0};
    double period{0.2};
    int max_step = 15;
    for (int step = 0; step < max_step; step++)
    {
        yaw = yaw + user_vel.angular.z * period;
        p.x = p.x + user_vel.linear.x * std::cos(yaw) * period;
        p.y = p.y + user_vel.linear.x * std::sin(yaw) * period;
        traj_marker.points.push_back(p);
    }
    traj_visualizer_pub_->publish(traj_marker);
}

void SharedControllerNode::main_process()
{
    // Check if new information from subscriber is received
    if (is_odom_updated_ && is_joystick_updated_ && is_gap_updated_)
    {
        geometry_msgs::msg::Twist cmd_vel;
        bool is_cmd_vel_found = false;
        auto user_vel = calculate_velocity_from_joy(joystick_);
        user_trajectory_visualization();
        if(abs(user_vel.linear.x) <= 1e-4 && abs(user_vel.angular.z) <= 1e-4) //Stop safely
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0; 
        }
        else{
            auto goal = convert_gap_to_pose(observed_gap_);
            geometry_msgs::msg::PoseStamped robot_pose;
            if(!costmap_ros_->getRobotPose(robot_pose))
            {
                RCLCPP_ERROR(get_logger(), "Failed to get robot pose");
                return;
            }
            geometry_msgs::msg::Twist robot_vel = odom_.twist.twist;
            nav_msgs::msg::Path path;
            computePathToPose(goal, path);
            try {
                if (validatePath(path)) {
                    cmd_vel = mppi_controller_->computeVelocityCommands(robot_pose, robot_vel, goal.pose, observed_gap_.confident, path, user_vel);
                    plan_publisher_->publish(path);
                }
                else {
                    cmd_vel = mppi_controller_->computeVelocityCommands(robot_pose, robot_vel, goal.pose, observed_gap_.confident, user_vel);
                }
            } catch (const std::runtime_error& e) {
                // Log the error message (optional)
                RCLCPP_ERROR(this->get_logger(), "Caught exception in computeVelocityCommands: %s", e.what());
                // Set cmd_vel to 0 if an exception occurs
                cmd_vel.linear.x = -0.05;
                cmd_vel.angular.z = 0; 
            }
        }

     
        // Publish the velocity command
        auto joy_cmd = calculate_joy_from_velocity(cmd_vel);
        joy_pub_->publish(joy_cmd);
        is_odom_updated_ = false;
        is_joystick_updated_ = false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SharedControllerNode>();
    node->configure();
    node->activate();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}