#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

Mirte_Sensor::Mirte_Sensor(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::vector<uint8_t> pins, std::string name) {
  this->tmx = tmx;
  this->nh = nh;
  this->pins = pins;
  this->name = name;
  this->board = board;
}

