cmake_minimum_required(VERSION 3.5)
project(exploration_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
ament_cmake
rclcpp
rclcpp_action
geometry_msgs
action_msgs
nav2_msgs
nav2_util
nav_msgs
rosidl_default_generators
sensor_msgs
tf2_ros
tf2
tf2_geometry_msgs
OpenCV
)


# INCLUDE
include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

set(include_dependencies
    include
)
# ~INCLUDE


# Add executable 1
add_executable(explore_sim src/helpers.cpp src/sim.cpp src/rosInterface.cpp)
ament_target_dependencies(explore_sim ${dependencies})

install(TARGETS explore_sim
DESTINATION lib/${PROJECT_NAME}
)

# launch params
# install(DIRECTORY include
# DESTINATION share/${PROJECT_NAME}
# )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
