cmake_minimum_required(VERSION 3.8)
project(homeplus_moveit_config)

find_package(ament_cmake REQUIRED)

# Find required dependencies
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/ik.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_kinematics
  moveit_planners_ompl
)

ament_package()
