#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "wheelchair_control_support/msg/gap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

using namespace std::chrono_literals;

class TestGlobalPlanner : public rclcpp::Node
{
public:
    TestGlobalPlanner()
  : Node("test_global_planner"), latest_gap_(nullptr)
  {
    std::cout << "Initializing TestGlobalPlanner node" << std::endl;
    // Create the action client for the "ComputePathToPose" action
    client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "compute_path_to_pose");

    // Wait for the action server to be available
    while (!client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }

    // Subscription to /intended_gap topic
    gap_sub_ = this->create_subscription<wheelchair_control_support::msg::Gap>(
      "/intended_gap", 1,
      std::bind(&TestGlobalPlanner::gap_callback, this, std::placeholders::_1)
    );

    // Create a timer to check the gap and request the action server periodically
    timer_ = this->create_wall_timer(
      50ms, std::bind(&TestGlobalPlanner::periodic_goal_request, this));
  }

private:
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_;
  rclcpp::Subscription<wheelchair_control_support::msg::Gap>::SharedPtr gap_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Store the latest gap received
  std::shared_ptr<wheelchair_control_support::msg::Gap> latest_gap_;

  // Callback function to handle new gaps
  void gap_callback(const wheelchair_control_support::msg::Gap::SharedPtr msg)
  {
    latest_gap_ = msg;  // Store the latest gap received
  }

  // Function to convert Gap to Pose
  geometry_msgs::msg::Pose convert_gap_to_pose(const wheelchair_control_support::msg::Gap & gap)
  {
    geometry_msgs::msg::Pose gap_pose{};
    gap_pose.position.x = gap.middle_point_x;
    gap_pose.position.y = gap.middle_point_y;
    gap_pose.position.z = 0.0;

    double angle = std::atan2(gap.left_point_x - gap.right_point_x, gap.right_point_y - gap.left_point_y);
    gap_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle);

    return gap_pose;
  }

  // Periodically check and send goal to the planner
  void periodic_goal_request()
  {
    if (latest_gap_ && latest_gap_->confident > 0.0) {
      // Convert gap to pose
      geometry_msgs::msg::Pose gap_pose = convert_gap_to_pose(*latest_gap_);

      // Create a PoseStamped message
      geometry_msgs::msg::PoseStamped goal_pose_stamped;
      goal_pose_stamped.header.stamp = this->get_clock()->now();
      goal_pose_stamped.header.frame_id = "base_footprint";  // Assuming global frame is "map"
      goal_pose_stamped.pose = gap_pose;

      // Send the goal to the action server
      send_goal_to_planner(goal_pose_stamped);
    }
  }

  // Function to send the goal to the planner action server
  void send_goal_to_planner(const geometry_msgs::msg::PoseStamped & goal_pose_stamped)
  {
    auto goal = nav2_msgs::action::ComputePathToPose::Goal();
    goal.goal = goal_pose_stamped;
    goal.planner_id =  "GridBased";

    // Send the goal asynchronously to the planner action server
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&TestGlobalPlanner::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&TestGlobalPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    client_->async_send_goal(goal, send_goal_options);
  }

  // Callback for result from the action server
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result)
  {
    // Access the planning_time which contains sec and nanosec components
    auto planning_time = result.result->planning_time;

    // Combine seconds and nanoseconds into a single floating-point value
    double total_time_in_seconds = planning_time.sec + planning_time.nanosec / 1e9;

    // Print the total planning time in seconds (as a float)
    RCLCPP_INFO(this->get_logger(), 
                "Received result from action server: %f seconds", 
                total_time_in_seconds);
  }

  // Callback for feedback from the action server
  void feedback_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback: Path planning in progress...");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestGlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
