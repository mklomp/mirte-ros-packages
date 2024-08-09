cmake_minimum_required(VERSION 3.9)
project(mirte_base_control)

set (CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(  hardware_interface REQUIRED)
find_package(  pluginlib REQUIRED)
find_package(   rclcpp REQUIRED)
find_package(  rclcpp_lifecycle REQUIRED) 
# find_package(controller_manager 4.0.0)

# catkin_package(
#   # INCLUDE_DIRS include/
# #  LIBRARIES mirte_ros_package
#   CATKIN_DEPENDS roscpp std_msgs mirte_msgs message_runtime controller_manager hardware_interface trajectory_msgs control_toolbox
# #  DEPENDS system_lib
# )

set(THIS_PACKAGE_INCLUDE_DEPENDS
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
control_toolbox
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()


include_directories(
 include/
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(${PROJECT_NAME}_node
    src/my_robot_base.cpp src/mirte_base_control.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}_node std_msgs std_srvs mirte_msgs rclcpp sensor_msgs    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  # DEPENDS
                                                              # system_lib
)
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# EXPORTS ament_export_targets( ${PROJECT_NAME}_node HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
ament_package()
