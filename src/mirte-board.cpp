#include <mirte-board.hpp>
#include <mirte-node.hpp>
#include <rclcpp/parameter.hpp>
Mirte_Board::Mirte_Board(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh) {
  this->tmx = tmx;
  this->nh = nh;
}

std::vector<uint8_t> Mirte_Board::resolvePins(rclcpp::Parameter keypad) {
  std::vector<uint8_t> pins;
  // if (keypad.hasMember("port")) {
  //   pins.push_back(1);
  // }
  // if (keypad.hasMember("pin")) {
  //   pins.push_back(2);
  // }
  pins.push_back(28);
  return pins;
}
