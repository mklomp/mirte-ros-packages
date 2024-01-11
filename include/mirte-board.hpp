#pragma once
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>

class Mirte_Board {
public:
  Mirte_Board(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  int get_adc_bits() { return 12; }
  std::vector<uint8_t> resolvePins(rclcpp::Parameter keypad);
};