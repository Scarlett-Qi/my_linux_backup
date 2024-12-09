cmake_minimum_required (VERSION 3.8)
project(gr_planning_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPath.srv"
  "srv/DetectPointCloud.srv"
  "srv/TestPath.srv"
  "action/ForkliftPath.action"
  "action/GetPath.action"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
