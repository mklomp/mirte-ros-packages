#include <mirte_telemetrix_cpp/parsers/sensors/intensity_data.hpp>

IntensityData::IntensityData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: SensorData(parser, board, name, IntensityData::get_device_class(), parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);

    // Mirte boards support not connecting either the digital or analog pin.
    if (pins.count("analog")) this->a_pin = pins["analog"];
    if (pins.count("digital")) this->d_pin = pins["digital"];
  } else if (unused_keys.erase("pins")) {
    if (parameters.count("pins.analog"))
      this->a_pin = board->resolvePin(get_string(parameters["pins.analog"]));

    if (parameters.count("pins.digital"))
      this->d_pin = board->resolvePin(get_string(parameters["pins.digital"]));
  } else
    RCLCPP_ERROR(
      logger, "Device %s.%s has no a connector or pins specified.", get_device_class().c_str(),
      name.c_str());
}

bool IntensityData::check()
{
  // Mirte boards support not connecting either the digital or analog pin.
  return (a_pin != (pin_t)-1 || d_pin != (pin_t)-1) && SensorData::check();
}