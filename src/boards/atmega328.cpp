#include <mirte-board.hpp>
#include <util.hpp>
Mirte_Board_atmega328p::Mirte_Board_atmega328p(
  std::shared_ptr<TMX> tmx,
  std::shared_ptr<rclcpp::Node> nh)
: Mirte_board(tmx, nh)
{
}

int Mirte_Board_atmega328p::resolvePin(std::string pin_name)
{
  int pin = -1;
  if ((pin = try_parse_int(pin_name)) != -1) {
    return pin;
  }
  if (starts_with(pin_name, "RX")) {
    return 0;
  }
  if (starts_with(pin_name, "TX")) {
    return 1;
  }
  if (starts_with(pin_name, "A")) {
    if (pin = try_parse_int(pin_name.substr(1)) != -1) {
      return pin + 14;
    }
  }
  if (starts_with(pin_name, "D")) {
    if (pin = try_parse_int(pin_name.substr(1)) != -1) {
      return std::clamp(pin, 0, 13);
    }
  }
  std::cerr << "Not implemented: atmega328p::resolvePin : " << pin_name << std::endl;
  return -1;
}
