#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "yaml-cpp/yaml.h"

void parse_params(std::string param_file,std::shared_ptr< rclcpp::Node> nh );