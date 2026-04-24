cmake_minimum_required(VERSION 3.8)
project(nova5_2f85_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

add_executable(grasp_task_planner_mtc_node
  src/grasp_task_planner_mtc_node.cpp
)

ament_target_dependencies(grasp_task_planner_mtc_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)

install(TARGETS
  grasp_task_planner_mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
