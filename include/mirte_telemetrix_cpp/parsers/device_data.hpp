#pragma once

#include <concepts>
#include <map>
#include <string>

#include <rclcpp/parameter.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class DeviceData
{
public:
  std::string name = "";
  std::string frame_id = "";

  DeviceData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters);
  DeviceData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::string device_type, std::map<std::string, rclcpp::ParameterValue> parameters);

  virtual bool check();

  /// @brief Get the device class of this type. So 'distance' for sonars and 'keypad' for keypads, etc.
  /// @return The device class string
  static std::string get_device_class() { return "no_type"; }
};

template <class T>
std::vector<typename std::enable_if<std::is_base_of<DeviceData, T>::value, T>::type> parse_all(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board)
{
  const auto device_class = T::get_device_class();

  std::vector<T> devices;
  for (auto name : parser->get_params_keys(device_class)) {
    auto parameters = parser->get_params_name(parser->build_param_name(device_class, name));

    auto data = T(parser, board, name, parameters);

    if (data.check())
      devices.push_back(data);
    else
      RCLCPP_ERROR(
        parser->nh->get_logger(), "%s device \"%s\" is invalid, skipping configuration.",
        device_class.c_str(), name.c_str());
  }

  return devices;
}