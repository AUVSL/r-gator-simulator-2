cmake_minimum_required(VERSION 3.5)

project(r_gator_gazebo)

find_package(ament_cmake REQUIRED COMPONENTS
  cv_bridge
  rclcpp
  rospy
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
)

## Your package locations should be listed before other locations
#ament_export_include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)


ament_package()

