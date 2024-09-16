#pragma once

#include <concepts>
#include <exception>
#include <map>
#include <string>

#include <rclcpp/parameter.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class SensorData
{
public:
  std::string name = "";
  std::string frame_id = "";

  SensorData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters);
  SensorData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::string sensor_type, std::map<std::string, rclcpp::ParameterValue> parameters);

  bool check();

  static std::string get_sensor_class() { return "no_type"; }
};

template <class T>
std::vector<typename std::enable_if<std::is_base_of<SensorData, T>::value, T>::type> parse_all(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board)
{
  const auto sensor_class = T::get_sensor_class();

  std::vector<T> sensors;
  for (auto name : parser->get_params_keys(sensor_class)) {
    auto parameters = parser->get_params_name(parser->build_param_name(sensor_class, name));

    auto data = T(parser, board, name, parameters);

    if (data.check())
      sensors.push_back(data);
    else
      RCLCPP_ERROR(
        parser->nh->get_logger(), "%s sensor \"%s\" is invalid, skipping configuration.",
        sensor_class.c_str(), name.c_str());
  }

  return sensors;
}