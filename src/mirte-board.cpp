#include <mirte-board.hpp>

std::shared_ptr<Mirte_Board> Mirte_Board::create(std::shared_ptr<Parser> parser)
{
  auto keys = parser->get_params_keys("device.mirte");
  auto values = parser->get_params_name("device.mirte");
  if (keys.count("type")) {
    auto type = get_string(values["type"]);
    if (type == "pcb") {
       

      Mirte_Board_pico pico;
      return std::make_shared<Mirte_Board_pcb>(std::make_shared<Mirte_Board_pico>(pico), get_string(values["version"]));
    } else if (type == "breadboard") {
      if (keys.count("board")) {
        auto board = get_string(values["board"]);
        if (board == "atmega328p") {
          return std::make_shared<Mirte_Board_atmega328p>();
        } else if (board == "pico") {
          return std::make_shared<Mirte_Board_pico>();
        } else {
          std::cerr << "Unknown board: " << board << std::endl;
        }
      } else {
        std::cerr << "No board specified" << std::endl;
      }
    } else {
      std::cerr << "Unknown board type: " << type << std::endl;
    }
  } else {
    std::cerr << "No board type specified" << std::endl;
  }
  return nullptr;
}

std::string get_string(rclcpp::ParameterValue param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return param.get<std::string>();
  } else {
     return rclcpp::to_string(param);
  }
}
