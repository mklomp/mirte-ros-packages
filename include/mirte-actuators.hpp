#pragma once
#include <mirte-board.hpp>
#include <mirte_msgs/SetMotorSpeed.h>
#include <ros/ros.h>
#include <tmx.hpp>
class Mirte_Actuator;
class Mirte_Actuators {
public:
  Mirte_Actuators(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board);
  TMX *tmx;
  ros::NodeHandle *nh;

  std::vector<Mirte_Actuator *> actuators;
  std::vector<uint8_t> resolvePins(XmlRpc::XmlRpcValue);
};

class Mirte_Actuator {
public:
  TMX *tmx;
  ros::NodeHandle *nh;
  Mirte_Board *board;
  std::vector<uint8_t> pins;
  virtual void publish() = 0;
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
  bool motor_callback(mirte_msgs::SetMotorSpeed::Request &req,
                      mirte_msgs::SetMotorSpeed::Response &res);
  int last_speed = 0;
  ros::Subscriber ros_client;
};