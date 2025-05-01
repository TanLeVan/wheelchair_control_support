cmake_minimum_required(VERSION 3.8)
project(wheelchair_control_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(wheelchair_control_support REQUIRED)
find_package(nav2_shared_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(wheelchair_smac_planner REQUIRED)

# Add Eigen as an external dependency
find_package(Eigen3 REQUIRED)

include_directories(include
  ./include/json/include/
)


# Adding custom message type
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gap.msg"
  DEPENDENCIES std_msgs
)


# Adding executatble
add_executable(test_global_planner src/test_global_planner.cpp)
ament_target_dependencies(test_global_planner
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  wheelchair_control_support
  nav2_msgs
  nav2_costmap_2d
  nav_2d_utils
)

add_executable(simple_velocity_publisher src/simple_velocity_publisher.cpp)
ament_target_dependencies(simple_velocity_publisher rclcpp geometry_msgs)

add_executable(share_control_node src/share_control_node.cpp)
ament_target_dependencies(share_control_node 
  rclcpp 
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  wheelchair_control_support
)
target_include_directories(share_control_node PUBLIC
  ./include
)

add_executable(shared_controller_node src/shared_controller_node.cpp)
ament_target_dependencies(shared_controller_node
  rclcpp 
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  wheelchair_control_support
  nav2_shared_mppi_controller
  nav2_costmap_2d
  nav_2d_utils
  wheelchair_smac_planner
)
target_include_directories(shared_controller_node PUBLIC
  ./include
  # ../nav2_shared_mppi_controller/include
)
# target_link_libraries(shared_controller_node nav2_shared_mppi_controller::shared_mppi_controller
# )


add_executable(intended_goal_estimator src/intended_goal_estimator.cpp)
ament_target_dependencies(intended_goal_estimator
  rclcpp
  nav_msgs
  sensor_msgs
  message_filters
  ament_index_cpp
  wheelchair_control_support
  visualization_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(intended_goal_estimator PUBLIC
  ./include
)

add_executable(footprint_visualizer src/test_visualization.cpp)
ament_target_dependencies(footprint_visualizer
  rclcpp
  visualization_msgs
)
target_include_directories(footprint_visualizer PUBLIC
  ./include
)

add_executable(calculate_acceleration src/cal_acceleration.cpp)
ament_target_dependencies(calculate_acceleration
  rclcpp
  nav_msgs
)
target_include_directories(calculate_acceleration PUBLIC
  ./include
)

add_executable(joy_control src/simple_joystick_publisher.cpp)
ament_target_dependencies(joy_control
  rclcpp
  sensor_msgs
)

add_executable(odometry_listener src/odometry_listener.cpp)
ament_target_dependencies(odometry_listener rclcpp nav_msgs)


add_executable(velocity_by_joystick_control_node src/vel_joy_exchange.cpp)
ament_target_dependencies(velocity_by_joystick_control_node rclcpp sensor_msgs geometry_msgs)

add_executable(test_find_gap src/test_find_gap.cpp)
ament_target_dependencies(test_find_gap rclcpp sensor_msgs geometry_msgs visualization_msgs)

add_executable(test_find_gap2 src/test_find_gap2.cpp)
ament_target_dependencies(test_find_gap2 rclcpp sensor_msgs geometry_msgs visualization_msgs)
target_include_directories(test_find_gap2 PUBLIC
  ./include
)

add_executable(test_egocircle_map src/test_egocircle_map.cpp)
ament_target_dependencies(test_egocircle_map 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
  nav_msgs
  Eigen3
)
target_include_directories(test_egocircle_map PUBLIC
  ./include
)

add_executable(scan_filter src/scan_filter.cpp)
ament_target_dependencies(scan_filter rclcpp sensor_msgs)
target_link_libraries(scan_filter ${PCL_LIBRARIES})



# Installing  

# ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS 
    simple_velocity_publisher 
    footprint_visualizer 
    calculate_acceleration
    shared_controller_node
    share_control_node
    joy_control
    odometry_listener
    velocity_by_joystick_control_node
    test_find_gap
    test_egocircle_map
    scan_filter
    test_find_gap2
    intended_goal_estimator
    test_global_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
	script/joy_visualizer.py
  script/test_scan_filter_py.py
  script/scan_filter.py
	DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY 
	config 
	launch
	model
  data
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
