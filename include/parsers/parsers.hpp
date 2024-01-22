#pragma once
#include "util.hpp"
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <set>
#include "rclcpp/rclcpp.hpp"
using pin_t = uint8_t;


class Parser {
public:
  std::shared_ptr<rclcpp::Node> nh;
  std::map<std::string, rclcpp::ParameterValue> params;
  Parser(std::shared_ptr<rclcpp::Node> nh);
  std::map<std::string, rclcpp::ParameterValue> get_params_name(std::string name);
  std::set<std::string> get_params_keys(std::string name);
  std::string build_param_name(std::string name, std::string key);
  int get_frequency() {
    return 50; // TODO: make this configurable
  }
};

std::string get_string(rclcpp::ParameterValue param);