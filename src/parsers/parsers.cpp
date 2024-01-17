#include "parsers/parsers.hpp"
#include <set>
std::map<std::string, rclcpp::ParameterValue> get_params_name(
  std::shared_ptr<rclcpp::Node> nh,
  std::string name)
{
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides =
    node_parameters_iface->get_parameter_overrides();
  decltype(parameter_overrides) out_params;
  for (auto & servo_it : parameter_overrides) {
//     std::cout << servo_it.first << std::endl;
    rclcpp::ParameterValue servo_config = servo_it.second;
    if (servo_it.first.rfind(name, 0) == 0) {
      out_params[servo_it.first] = servo_config;
    }
  }
  return out_params;
}

std::set<std::string> get_params_key_names(
  std::shared_ptr<rclcpp::Node> nh,
  std::string name)
{
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides =
    node_parameters_iface->get_parameter_overrides();
  std::set<std::string> out_params;
  for (auto & servo_it : parameter_overrides) {
//     std::cout << servo_it.first << std::endl;
    rclcpp::ParameterValue servo_config = servo_it.second;

    if (servo_it.first.rfind(name, 0) == 0) {
      std::cout << servo_it.first << std::endl;
      std::string key = servo_it.first.substr(name.length() + 1);
      std::cout << "1:" << key << std::endl;
      auto next_dot = key.find(".");
      key = key.substr(0, next_dot);
      std::cout << "2:" << key << "," << next_dot << std::endl;

      out_params.insert(key);
    }
  }
  return out_params;
}


Parser::Parser(std::shared_ptr<rclcpp::Node> nh)
{
  this->nh = nh;
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides =
    node_parameters_iface->get_parameter_overrides();
  this->params = parameter_overrides;
}

std::map<std::string, rclcpp::ParameterValue> Parser::get_params_name(std::string name)
{
  std::map<std::string, rclcpp::ParameterValue> out_params;
  for (auto & servo_it : this->params) {
    if (servo_it.first.rfind(name, 0) == 0) {
      out_params[servo_it.first] = servo_it.second;
    }
  }
  return out_params;
}

std::set<std::string> Parser::get_params_keys(std::string name)
{
  std::set<std::string> out_params;
  for (auto & servo_it : this->params) {
    if (servo_it.first.rfind(name, 0) == 0) {
      std::string key = servo_it.first.substr(name.length() + 1);
      auto next_dot = key.find(".");
      key = key.substr(0, next_dot);
      out_params.insert(key);
    }
  }
  return out_params;
}
