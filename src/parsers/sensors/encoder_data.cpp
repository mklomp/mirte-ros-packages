#include <mirte_telemetrix_cpp/parsers/sensors/encoder_data.hpp>

EncoderData::EncoderData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: SensorData(parser, board, name, EncoderData::get_device_class(), parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->pinA = pins["pinA"];
    this->pinB = pins["pinB"];
  } else if (unused_keys.erase("pins")) {
    // FIXME: Maybe restructure to test if pin A and B or only pin is set.
    if (parameters.count("pins.A"))
      this->pinA = board->resolvePin(get_string(parameters["pins.A"]));

    if (parameters.count("pins.B"))
      this->pinB = board->resolvePin(get_string(parameters["pins.B"]));

    if (parameters.count("pins.pin")) {
      this->pinA = board->resolvePin(get_string(parameters["pins.pin"]));
      this->pinB = (pin_t)-1;
    }
  } else
    RCLCPP_ERROR(
      logger, "Device %s.%s has no a connector or pins specified.", get_device_class().c_str(),
      name.c_str());
}

bool EncoderData::check()
{
  // Don't check pinB, since this could be a single pin encoder.
  return pinA != (pin_t)-1 && SensorData::check();
}