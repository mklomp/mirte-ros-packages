#include "mirte-board.hpp"

Mirte_Board_pico::Mirte_Board_pico(
  // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh
  )
// : Mirte_Board(tmx, nh)
{

}

int Mirte_Board_pico::resolvePin(std::string pin_name)
{
  int pin = -1;
  if ((pin = try_parse_int(pin_name)) != -1) {
    return pin;
  }
  if (pin_name.rfind("GP", 0) == 0) {
    if (pin = try_parse_int(pin_name.substr(2)) != -1) {
      return pin;
    }
  }
  std::cerr << "Not implemented: pico::resolvePin : " << pin_name << std::endl;
  return -1;
}

std::map<std::string, int> Mirte_Board_pico::resolveConnector(std::string connector)
{
  std::cerr << "Not implemented: pico::resolveConnector : " << connector << std::endl;
  return {};
}
