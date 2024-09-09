#pragma once
#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <mirte_msgs/srv/set_pin_value.hpp>

#include "parsers/actuators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <mirte-board.hpp>
#include <string>
#include <tmx_cpp/tmx.hpp>
#include <vector>

class Mirte_Actuator;
class Mirte_Actuators {
public:
  Mirte_Actuators(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                  std::shared_ptr<Mirte_Board> board,
                  std::shared_ptr<Parser> parser);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<Mirte_Actuator>> actuators;

  rclcpp::Service<mirte_msgs::srv::SetPinValue>::SharedPtr set_pin_value_service;

private:
  void set_pin_value_service_callback(const mirte_msgs::srv::SetPinValue::Request::ConstSharedPtr req,
                                      mirte_msgs::srv::SetPinValue::Response::SharedPtr res);
};

class Mirte_Actuator {
public:
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<pin_t> pins;
  std::string name;
  auto get_header() {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Actuator(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                 std::shared_ptr<Mirte_Board> board, std::vector<pin_t> pins,
                 std::string name);
  virtual ~Mirte_Actuator() {}
};

#include <actuators/servo.hpp>

class Motor : public Mirte_Actuator {
public:
  Motor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
        std::shared_ptr<Mirte_Board> board,
        std::shared_ptr<Motor_data> motor_data, std::string name,
        std::vector<pin_t> pins);
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;
  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
      std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res);
  void motor_callback(const std_msgs::msg::Int32 &msg);
  int last_speed = 0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ros_client;
  static std::vector<std::shared_ptr<Mirte_Actuator>>
  get_motors(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
             std::shared_ptr<Mirte_Board> board,
             std::shared_ptr<Parser> parser);
  virtual void set_speed(int speed) = 0;
  std::shared_ptr<Motor_data> motor_data;
  int max_pwm;
  void start() { this->set_speed(0); }
};

class PPMotor : public Motor {
public:
  PPMotor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
          std::shared_ptr<Mirte_Board> board,
          std::shared_ptr<Motor_data> motor_data, std::string name);

  PPMotor(); // Only for PCA_motor
  void set_speed(int speed);
  pin_t pwmA_pin;
  pin_t pwmB_pin;
  void setA(int speed);
  void setB(int speed);
};

class DPMotor : public Motor {
public:
  DPMotor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
          std::shared_ptr<Mirte_Board> board,
          std::shared_ptr<Motor_data> motor_data, std::string name);
  void set_speed(int speed);
  pin_t dir_pin;
  pin_t pwm_pin;
};

class DDPMotor : public Motor {
public:
  DDPMotor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
           std::shared_ptr<Mirte_Board> board,
           std::shared_ptr<Motor_data> motor_data, std::string name);
  void set_speed(int speed);
  pin_t A_pin;
  pin_t B_pin;
  pin_t pwm_pin;
};
