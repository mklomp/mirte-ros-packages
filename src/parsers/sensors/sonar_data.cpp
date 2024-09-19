#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>

SonarData::SonarData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: SensorData(parser, board, name, SonarData::get_device_class(), parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->trigger = pins["trigger"];
    this->echo = pins["echo"];
  } else if (unused_keys.erase("pins")) {
    if (parameters.count("pins.trigger"))
      this->trigger = board->resolvePin(get_string(parameters["pins.trigger"]));

    if (parameters.count("pins.echo"))
      this->echo = board->resolvePin(get_string(parameters["pins.echo"]));
  } else
    RCLCPP_ERROR(
      logger, "Device %s . %s has no a connector or pins specified.", get_device_class().c_str(),
      name.c_str());
}

bool SonarData::check() { return trigger != (pin_t)-1 && echo != (pin_t)-1 && SensorData::check(); }

/* TODO: REMOVE
std::vector<SonarData> parse_sonars(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board)
{
  std::vector<SonarData> sonars;
  for (auto name : parser->get_params_keys("distance")) {
    auto parameters = parser->get_params_name(parser->build_param_name("distance", name));

    auto data = SonarData(parser, board, name, parameters);

    if (data.check())
      sonars.push_back(data);
    else
      RCLCPP_ERROR(
        parser->nh->get_logger(), "Sonar \"%s\" is invalid, skipping confinguration", name.c_str());
  }

  return sonars;
}
*/
