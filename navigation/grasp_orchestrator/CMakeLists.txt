cmake_minimum_required(VERSION 3.8)
project(grasp_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)


include_directories(include)

add_executable(grasp_orchestrator
  src/grasp_orchestrator.cpp
  src/grasp_orchestrator_node.cpp
)

ament_target_dependencies(grasp_orchestrator
  rclcpp
  nav2_msgs
  apriltag_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

install(TARGETS
  grasp_orchestrator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
