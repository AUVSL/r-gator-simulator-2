cmake_minimum_required(VERSION 3.5)
project(r-gator-gazebo-tool-2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs)

include_directories(
  include
  /opt/ros/humble/include/nav_msgs
  /opt/ros/humble/include/tf2_geometry_msgs
)

# Declare a C++ executable
add_executable(gazebo_odom_publisher_node src/gazebo_odom_publisher.cpp)
add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(gazebo_odom_publisher_node
  rclcpp
  std_msgs
  ackermann_msgs
  tf2
  tf2_ros
  nav_msgs
  gazebo_msgs
)



ament_target_dependencies(tf_broadcaster_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav_msgs
)

# Install executables and/or libraries
install(TARGETS
  gazebo_odom_publisher_node
  tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
