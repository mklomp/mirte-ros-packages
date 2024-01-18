#include "mirte-board.hpp"

// Mirte_Board_pico::Mirte_Board_pico(
//   // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh
//   )
// // : Mirte_Board(tmx, nh)
// {

// }

Mirte_Board_pico::Mirte_Board_pico()
{

}

int Mirte_Board_pico::resolvePin(std::string pin_name)
{
  std::cout << "Mirte_Board_pico::resolvePin" << pin_name << std::endl;
  // int pin = -1;
  if (auto pin = try_parse_int(pin_name) ) {
    std::cout << "Mirte_Board_pico::tryparse" << *pin << std::endl;
    return pin.value();
  }
  if (starts_with(pin_name,"GP")) {
    if (auto pin = try_parse_int(pin_name.substr(2))  ) {
      return pin.value();
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
