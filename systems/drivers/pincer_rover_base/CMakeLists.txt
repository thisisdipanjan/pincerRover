cmake_minimum_required(VERSION 3.8)
project(pincer_rover_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(SOURCES
  src/main.cpp
  src/ackermann_node.cpp
  src/odometry_calculator.cpp
  src/i2c_queue.cpp
)

add_executable(pincer_rover_base_node ${SOURCES})

target_include_directories(pincer_rover_base_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pincer_rover_base_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(pincer_rover_base_node i2c)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS pincer_rover_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
