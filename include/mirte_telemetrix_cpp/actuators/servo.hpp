#pragma once
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/servo_data.hpp>
#include "mirte_telemetrix_cpp/mirte-actuators.hpp"

class Servo : public Mirte_Actuator
{
public:
  Servo(NodeData node_data, ServoData servo_data);

  // TODO: Maybe make private
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr set_angle_service;
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr get_range_service;

  static std::vector<std::shared_ptr<Servo>> get_servos(
    NodeData node_data, std::shared_ptr<Parser> parser);

  ~Servo();

private:
  void set_angle_service_callback(
    const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngle::Response::SharedPtr res);

  void get_range_service_callback(
    const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetServoRange::Response::SharedPtr res);

  ServoData data;

  // TODO: Maybe add deregister just like Python version
};