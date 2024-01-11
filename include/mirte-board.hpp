#pragma once
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>
#include <xmlrpcpp/XmlRpcValue.h>
class Mirte_Board {
public:
  Mirte_Board(TMX &tmx, ros::NodeHandle &nh);
  TMX *tmx;
  ros::NodeHandle *nh;
  int get_adc_bits() { return 12; }
  std::vector<uint8_t> resolvePins(XmlRpc::XmlRpcValue keypad);
};