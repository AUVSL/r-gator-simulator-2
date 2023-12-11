cmake_minimum_required(VERSION 3.8)
project(r-gator-teleop-2)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add any other dependencies you need here
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(r_gator_teleop_node src/r_gator_teleop.cpp)
install(
  PROGRAMS
  src/r_gator_teleop.cpp
  DESTINATION lib/${PROJECT_NAME}
)


ament_target_dependencies(r_gator_teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  # Add any other dependencies used in your project
)

ament_package()
