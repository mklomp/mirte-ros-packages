#include <mirte_telemetrix_cpp/parsers/sensors/keypad_data.hpp>

KeypadData::KeypadData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: SensorData(parser, board, name, KeypadData::get_device_class(), parameters)
{
  if (parameters.count("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->pin = pins["pin"];
  } else {
    if (parameters.count("pins")) this->pin = board->resolvePin(get_string(parameters["pins.pin"]));
  }
}

bool KeypadData::check() { return pin != (pin_t)-1 && SensorData::check(); }