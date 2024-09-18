#include <mirte_telemetrix_cpp/parsers/actuators/servo_data.hpp>

ServoData::ServoData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: DeviceData(parser, board, name, ServoData::get_device_class(), parameters)
{
  if (parameters.count("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);

    this->pin = pins["pin"];
  } else {
    if (parameters.count("pins.pin"))
      this->pin = board->resolvePin(get_string(parameters["pins.pin"]));
  }

  if (parameters.count("min_pulse")) this->min_pulse = parameters["min_pulse"].get<int>();

  if (parameters.count("max_pulse")) this->max_pulse = parameters["max_pulse"].get<int>();

  if (parameters.count("min_angle")) this->min_angle = parameters["min_angle"].get<float>();

  if (parameters.count("max_angle")) this->max_angle = parameters["max_angle"].get<float>();
}

bool ServoData::check() { return pin != (pin_t)-1 && DeviceData::check(); }