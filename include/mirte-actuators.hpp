#pragma once
#include <mirte-board.hpp>
#include "mirte_msgs/srv/set_motor_speed.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include <tmx.hpp>
class Mirte_Actuator;
class Mirte_Actuators
{
public:
  Mirte_Actuators(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<Mirte_Actuator *> actuators;
};

class Mirte_Actuator
{
public:
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<uint8_t> pins;
  std::string name;
  auto get_header()
  {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Actuator(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
};

class Motor : public Mirte_Actuator
{
public:
  Motor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;
  bool service_callback(
   const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res);
  void motor_callback(const std_msgs::msg::Int32 & msg);
  int last_speed = 0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ros_client;
  static std::vector<Mirte_Actuator *> get_motors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board);
  virtual void set_speed(int speed) = 0;
};

class PPMotor : public Motor
{
public:
  PPMotor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};

class DPMotor : public Motor
{
public:
  DPMotor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};

class DDPMotor : public Motor
{
public:
  DDPMotor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, 
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};
