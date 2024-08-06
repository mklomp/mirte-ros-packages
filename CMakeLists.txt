cmake_minimum_required(VERSION 3.9)
project(mirte_telemetrix_cpp)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++20)
set(CMAKE_CXX_STANDARD 20)

# find_program(iwyu_path NAMES include-what-you-use iwyu REQUIRED) if(NOT
# iwyu_path) message(FATAL_ERROR "Could not find the program
# include-what-you-use") endif()

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include/ libs/tmx-cpp/include/
                    libs/tmx-cpp/libs/async_serial/ ${catkin_INCLUDE_DIRS})

# Declare a C++ library add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/mobile_controller.cpp )

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure add_dependencies(${PROJECT_NAME}
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide
include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++ -Wswitch-enum") # -Wall -Wextra
                                                                 # -Wpedantic
                                                                 # -Werror")

add_executable(
  ${PROJECT_NAME}_node
  # src/test.cpp
  src/mirte-telemetrix.cpp
  src/mirte-board.cpp
  src/boards/atmega328.cpp
  src/boards/pico.cpp
  src/boards/pcb.cpp
  src/mirte-actuators.cpp
  src/mirte-modules.cpp
  src/mirte-sensors.cpp
  src/mirte-ping.cpp
  src/parsers/actuators.cpp
  src/parsers/parsers.cpp
  src/parsers/p_modules.cpp
  src/parsers/p_sensors.cpp
  libs/tmx-cpp/src/tmx.cpp
  libs/tmx-cpp/src/buffer_build.cpp
  libs/tmx-cpp/src/sensors.cpp
  libs/tmx-cpp/src/modules.cpp
  libs/tmx-cpp/src/modules/PCA9685.cpp
  libs/tmx-cpp/src/modules/HiwonderServo.cpp
  libs/tmx-cpp/src/sensors/INA226.cpp
  libs/tmx-cpp/libs/async_serial/AsyncSerial.cpp
  libs/tmx-cpp/libs/async_serial/BufferedAsyncSerial.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node std_msgs std_srvs mirte_msgs rclcpp sensor_msgs # DEPENDS
                                                              # system_lib
)
find_package(Boost REQUIRED COMPONENTS system thread)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS} )

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Boost::system
                      Boost::thread)
# set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_INCLUDE_WHAT_YOU_USE
# ${iwyu_path})

# if(supported) if(CMAKE_BUILD_TYPE STREQUAL "Release") message(STATUS "IPO /
# LTO enabled") set_property(TARGET ${PROJECT_NAME}_node PROPERTY
# INTERPROCEDURAL_OPTIMIZATION TRUE) endif() else() message(STATUS "IPO / LTO
# not supported: <${error}>") endif()

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
