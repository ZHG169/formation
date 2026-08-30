cmake_minimum_required(VERSION 3.8)
project(formation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FormationStatus.msg"
  "msg/FormationError.msg"
  "msg/FormationCommand.msg"
  "srv/SetLeader.srv"
  "srv/SetFormation.srv"
  DEPENDENCIES builtin_interfaces
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# rosidl already creates the Python package target named "formation".
# Install the handwritten modules into the same package directory
# without creating a second ament_cmake_python target.
install(
  DIRECTORY formation/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(
  PROGRAMS
    scripts/formation_node
    scripts/mission_node
    scripts/leader_command_node
    scripts/leader_control_node
    scripts/follower_control_node
    scripts/distributed_vehicle_node
    scripts/monitor_mav_dds.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()