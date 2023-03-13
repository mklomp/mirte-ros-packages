cmake_minimum_required(VERSION 3.8)
project(mirte_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Encoder.msg"
  "msg/Intensity.msg"
  "msg/IntensityDigital.msg"
  "msg/Keypad.msg"
)

set(srv_files
  "srv/GetDistance.srv"
  "srv/GetEncoder.srv"
  "srv/GetIntensity.srv"
  "srv/GetIntensityDigital.srv"
  "srv/GetKeypad.srv"
  "srv/GetPinValue.srv"
  "srv/SetLEDValue.srv"
  "srv/SetMotorSpeed.srv"
  "srv/SetOLEDImage.srv"
  "srv/SetPinMode.srv"
  "srv/SetPinValue.srv"
  "srv/SetServoAngle.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
