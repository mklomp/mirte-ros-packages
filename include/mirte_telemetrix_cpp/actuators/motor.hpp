#pragma once
#include <rclcpp/rclcpp.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>

#include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/mirte-actuators.hpp>
#include <mirte_telemetrix_cpp/parsers/actuators/motor_data.hpp>

class Motor : public Mirte_Actuator
{
public:
  Motor(
    NodeData node_data, MotorData motor_data, std::vector<pin_t> pins);
  
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;
  
  bool service_callback(
    const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res);
  
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

class PPMotor : public Motor
{
public:
  PPMotor(NodeData node_data, MotorData motor_data);

  // PPMotor();  // Only for PCA_motor
  void set_speed(int speed);
  pin_t pwmA_pin;
  pin_t pwmB_pin;
  void setA(int speed);
  void setB(int speed);
};

class DPMotor : public Motor
{
public:
  DPMotor(NodeData node_data, MotorData motor_data);
  void set_speed(int speed);
  pin_t dir_pin;
  pin_t pwm_pin;
};

class DDPMotor : public Motor
{
public:
  DDPMotor(NodeData node_data, MotorData motor_data);
  void set_speed(int speed);
  pin_t A_pin;
  pin_t B_pin;
  pin_t pwm_pin;
};
