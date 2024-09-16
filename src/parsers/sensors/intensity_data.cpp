#include <mirte_telemetrix_cpp/parsers/sensors/intensity_data.hpp>

IntensityData::IntensityData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: SensorData(parser, board, name, IntensityData::get_sensor_class(), parameters)
{
  if (parameters.count("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);

    // Mirte boards support not connecting either the digital or analog pin.
    if (pins.count("analog")) this->a_pin = pins["analog"];
    if (pins.count("digital")) this->d_pin = pins["digital"];
  } else {
    if (parameters.count("pins.analog"))
      this->a_pin = board->resolvePin(get_string(parameters["pins.analog"]));

    if (parameters.count("pins.digital"))
      this->d_pin = board->resolvePin(get_string(parameters["pins.digital"]));
  }
}

bool IntensityData::check()
{
  // Mirte boards support not connecting either the digital or analog pin.
  return (a_pin != (pin_t)-1 || d_pin != (pin_t)-1) && SensorData::check();
}