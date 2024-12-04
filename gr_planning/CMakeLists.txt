cmake_minimum_required(VERSION 3.8)
project(gr_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gr_planning_interface REQUIRED)
find_package(ridar_service_interface REQUIRED)
find_package(mima_agv_services REQUIRED)

add_executable(gr_planning_forklift_correct_node src/ForkliftCorrectNode.cpp)

target_include_directories(gr_planning_forklift_correct_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GTSAM_INCLUDE_DIR}
  )

ament_target_dependencies(gr_planning_forklift_correct_node rclcpp std_msgs nav_msgs)

target_link_libraries(gr_planning_forklift_correct_node
  ${cpp_typesupport_target}
  gtsam
  gtsam_unstable
)

###### Path Server ##########
add_executable(gr_planning_forklift_correct_server src/ForkliftCorrectServer.cpp)

target_include_directories(gr_planning_forklift_correct_server PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(gr_planning_forklift_correct_server
  rclcpp
  rclcpp_action
  nav_msgs
  std_msgs
  visualization_msgs
  gr_planning_interface
)

target_link_libraries(gr_planning_forklift_correct_server
  ${gr_planning_forklift_correct_server}
  ${OpenCV_LIBS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

####### Path Client ##########
add_executable(PlanningPath test/ForkliftPathTest/ForkliftPathClient.cpp)

target_include_directories(PlanningPath PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(PlanningPath
  rclcpp
  nav_msgs
  gr_planning_interface
  ridar_service_interface
)

target_link_libraries(PlanningPath
  ${PlanningPath}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

######## Forklift Client ########
add_executable(ForkliftClient test/ForkliftTest/ForkliftClient.cpp)

target_include_directories(ForkliftClient PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(ForkliftClient
  rclcpp
  gr_planning_interface
  ridar_service_interface
)

target_link_libraries(ForkliftClient
  ${ForkliftClient}
)

########### Forklift Path Action Server ##########
add_executable(forklift_action_server src/ForkliftPathActionServer.cpp)

target_include_directories(forklift_action_server PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(forklift_action_server
  rclcpp
  rclcpp_action
  gr_planning_interface
  ridar_service_interface
  mima_agv_services
)

target_link_libraries(forklift_action_server
  ${forklift_action_server}
)

######### Forklift Path Action Client ###########
add_executable(forklift_action_client test/ForkliftTest/ForkliftActionClient.cpp)

target_include_directories(forklift_action_client PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(forklift_action_client
  rclcpp
  rclcpp_action
  gr_planning_interface
  mima_agv_services
)

target_link_libraries(forklift_action_client
  ${forklift_action_client}
)

#####################################

install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  gr_planning_forklift_correct_node
  gr_planning_forklift_correct_server
  PlanningPath
  ForkliftClient
  forklift_action_server
  forklift_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
