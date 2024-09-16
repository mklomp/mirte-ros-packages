#include <mirte_telemetrix_cpp/parsers/parsers.hpp>
#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

SensorData::SensorData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board>, std::string name,
  std::string sensor_type, std::map<std::string, rclcpp::ParameterValue> parameters)
: name(name)
{
  if (parameters.count("name")) assert(this->name.compare(get_string(parameters["name"])) == 0);

  std::string frame_prefix =
    (parser->params.count("frame_prefix") && !get_string(parser->params["frame_prefix"]).empty())
      ? get_string(parser->params["frame_prefix"])
      : "";

  if (parameters.count("frame_id")) {
    this->frame_id = frame_prefix + get_string(parameters["frame_id"]);
  } else {
    this->frame_id = frame_prefix + sensor_type + "_" + name;
  }
}

SensorData::SensorData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: SensorData(parser, board, name, this->get_sensor_class(), parameters)
{
}

bool SensorData::check() { return name != ""; }