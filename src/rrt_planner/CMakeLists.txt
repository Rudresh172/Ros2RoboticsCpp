cmake_minimum_required(VERSION 3.8)
project(rrt_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(rrt_planner_node src/rrt_planner.cpp)
ament_target_dependencies(rrt_planner_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  rrt_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
