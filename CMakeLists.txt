cmake_minimum_required(VERSION 3.9)
project(mirte_telemetrix_cpp)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++20)
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

# find_program(iwyu_path NAMES include-what-you-use iwyu REQUIRED) if(NOT
# iwyu_path) message(FATAL_ERROR "Could not find the program
# include-what-you-use") endif()

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
add_subdirectory(libs/tmx-cpp)
include_directories(include/ libs/tmx-cpp/include/
                    libs/tmx-cpp/libs/async_serial/include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum -Wall -Werror -Wno-sign-compare") # -Wall -Wextra
                                                                 # -Wpedantic
                                                                 # -Werror")
# set(NO_CLANG_TIDY FALSE CACHE BOOL "Disable clang-tidy")
# if(NOT NO_CLANG_TIDY)
# set(CMAKE_CXX_CLANG_TIDY
# clang-tidy-14;
# -format-style='file';
# -header-filter=${CMAKE_CURRENT_SOURCE_DIR};
# )
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# endif()

add_executable(
  ${PROJECT_NAME}_node
  src/util.cpp
  src/mirte-telemetrix.cpp
  src/mirte-board.cpp
  src/boards/atmega328.cpp
  src/boards/pico.cpp
  src/boards/pcb.cpp
  src/mirte-actuators.cpp
  src/mirte-modules.cpp
  src/mirte-sensors.cpp
  src/mirte-ping.cpp
  src/device.cpp
  src/actuators/servo.cpp
  src/actuators/motor.cpp
  src/parsers/parsers.cpp
  src/parsers/p_modules.cpp
  src/parsers/device_data.cpp
  src/parsers/actuators/motor_data.cpp
  src/parsers/actuators/servo_data.cpp
  src/parsers/sensors/base_sensor_data.cpp
  src/parsers/sensors/encoder_data.cpp
  src/parsers/sensors/intensity_data.cpp
  src/parsers/sensors/sonar_data.cpp
  src/parsers/sensors/keypad_data.cpp
  src/parsers/modules/module_data.cpp
  src/parsers/modules/i2c_module_data.cpp
  src/parsers/modules/ina226_data.cpp
  src/parsers/modules/hiwonder_data.cpp
  src/parsers/modules/pca_data.cpp
  src/sensors/base_sensor.cpp
  src/sensors/encoder_monitor.cpp
  src/sensors/intensity_monitor.cpp
  src/sensors/keypad_monitor.cpp
  src/sensors/sonar_monitor.cpp
  src/modules/base_module.cpp
  src/modules/pca_module.cpp
  src/modules/ina226_module.cpp
  src/modules/hiwonder_module.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node std_msgs std_srvs mirte_msgs rclcpp rcpputils sensor_msgs # DEPENDS
                                                              # system_lib
)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS} )

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Boost::system
                      Boost::thread tmx_cpp)
set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_INCLUDE_WHAT_YOU_USE
${iwyu_path})

if(supported)
 if(CMAKE_BUILD_TYPE STREQUAL "Release") 
 message(STATUS "IPO LTO enabled") 
 set_property(TARGET ${PROJECT_NAME}_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE) 
 endif() 
 else() 
 message(STATUS "IPO  / LTO not supported: <${error}>") 
 endif()

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
