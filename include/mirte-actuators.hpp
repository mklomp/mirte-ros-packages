#pragma once
#include <mirte-board.hpp>
#include <mirte_msgs/Keypad.h>
#include <mirte_msgs/SetMotorSpeed.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/Int32.h>
#include <tmx.hpp>
class Mirte_Actuator;
class Mirte_Actuators {
public:
  Mirte_Actuators(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board);
  TMX *tmx;
  ros::NodeHandle *nh;
  Mirte_Board *board;
  std::vector<Mirte_Actuator *> actuators;
};

class Mirte_Actuator {
public:
  TMX *tmx;
  ros::NodeHandle *nh;
  Mirte_Board *board;
  std::vector<uint8_t> pins;
  std::string name;
  auto get_header() {
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    return header;
  }
  Mirte_Actuator(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
                 std::vector<uint8_t> pins, std::string name);
};

class Motor : public Mirte_Actuator {
public:
  Motor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
        std::vector<uint8_t> pins, std::string name);
  ros::ServiceServer motor_service;
  bool service_callback(mirte_msgs::SetMotorSpeed::Request &req,
                        mirte_msgs::SetMotorSpeed::Response &res);
  void motor_callback(const std_msgs::Int32 &msg);
  int last_speed = 0;
  ros::Subscriber ros_client;
  static std::vector<Mirte_Actuator *> get_motors(ros::NodeHandle &nh, TMX &tmx,
                                                  Mirte_Board &board);
  virtual void set_speed(int speed) = 0;
};

class PPMotor : public Motor {
public:
  PPMotor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
          std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};

class DPMotor : public Motor {
public:
  DPMotor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
          std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};

class DDPMotor : public Motor {
public:
  DDPMotor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
           std::vector<uint8_t> pins, std::string name);
  void set_speed(int speed);
};