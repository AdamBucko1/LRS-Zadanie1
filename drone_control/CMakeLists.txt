cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(PCL 1.2 REQUIRED)

# Declare a C++ executable
#add_executable(drone_control_node 
#              src/drone_control_node.cpp
#              src/map_handler.cpp
#)
add_executable(point_cloud_node 
               src/point_cloud_handler.cpp
) 

#ament_target_dependencies(drone_control_node rclcpp geometry_msgs mavros_msgs)
ament_target_dependencies(point_cloud_node rclcpp geometry_msgs mavros_msgs)

#install(TARGETS drone_control_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_node DESTINATION lib/${PROJECT_NAME})


include_directories(
  include
  /opt/ros/foxy/include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (drone_control_node ${PCL_LIBRARIES})
target_link_libraries (point_cloud_node ${PCL_LIBRARIES})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
