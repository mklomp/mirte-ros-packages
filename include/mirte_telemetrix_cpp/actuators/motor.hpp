#pragma once
#include <memory>
#include <vector>

#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <mirte_telemetrix_cpp/mirte-actuators.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/parsers/actuators/motor_data.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <std_msgs/msg/int32.hpp>

class Motor : public Mirte_Actuator
{
public:
  Motor(NodeData node_data, std::vector<pin_t> pins, MotorData motor_data);

  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;

  bool service_callback(
    const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res);

  void motor_callback(const std_msgs::msg::Int32 & msg);

  int last_speed = 0;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ros_client;

  static std::vector<std::shared_ptr<Mirte_Actuator>> get_motors(
    NodeData node_data, std::shared_ptr<Parser> parser);

  virtual void set_speed(int speed) = 0;

  MotorData data;
  int max_pwm;

  // TODO: Maybe add start to TelemetrixDevice
  void start() { this->set_speed(0); }
};
