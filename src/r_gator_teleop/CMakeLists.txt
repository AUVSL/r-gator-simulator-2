cmake_minimum_required(VERSION 3.5)
project(r_gator_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rospy REQUIRED)
find_package(sensor_msgs REQUIRED
)


## Your package locations should be listed before other locations
#ament_export_include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)

ament_package()