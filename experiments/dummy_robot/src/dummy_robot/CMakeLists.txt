cmake_minimum_required(VERSION 3.8)
project(dummy_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(dummy_robot_node
  src/dummy_robot_node.cpp
  src/lanelet_router.cpp
)
target_include_directories(dummy_robot_node PRIVATE include)
ament_target_dependencies(dummy_robot_node
  rclcpp nav_msgs std_msgs sensor_msgs geometry_msgs
  lanelet2_core lanelet2_io lanelet2_projection lanelet2_routing lanelet2_traffic_rules
)
target_link_libraries(dummy_robot_node yaml-cpp)

install(
  TARGETS dummy_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
