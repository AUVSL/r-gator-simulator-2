cmake_minimum_required(VERSION 3.8)
project(r-gator-description-2)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# Add more dependencies as needed, for example:
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# etc...

# Include directories
include_directories(include)

# If the package has libraries or executables to build
# add_library(${PROJECT_NAME}_lib src/my_lib.cpp)
# add_executable(${PROJECT_NAME}_node src/my_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Install executables and/or libraries
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install config files

# Uncomment if the package has a setup.py
# ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export information to downstream packages
ament_package()
