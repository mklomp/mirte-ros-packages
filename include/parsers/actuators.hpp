#pragma once
#include "parsers/parsers.hpp"

class Servo_data
{
public:
  std::string name;
  pin_t pin;
  std::string connector;
  int min_pulse;
  int max_pulse;
};

std::vector<std::shared_ptr<Servo_data>> parse_servo_data(std::shared_ptr<rclcpp::Node> nh);