cmake_minimum_required(VERSION 3.8)
project(lattice_planner_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ae_hyu_msgs REQUIRED)
find_package(obstacle_detection_pkg REQUIRED)

# Lattice Planner Node
add_executable(lattice_planner_node 
  src/lattice_planner.cpp
  src/utils.cpp
)

ament_target_dependencies(lattice_planner_node
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ae_hyu_msgs
  obstacle_detection_pkg
)

target_include_directories(lattice_planner_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install targets
install(TARGETS 
        lattice_planner_node
        DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
        DESTINATION include/)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

ament_package()