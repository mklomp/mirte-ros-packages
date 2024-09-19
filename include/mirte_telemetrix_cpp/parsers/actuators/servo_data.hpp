#pragma once

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

class ServoData : public DeviceData
{
public:
  pin_t pin = (pin_t)-1;
  int min_pulse = 544;
  int max_pulse = 2400;
  // Min/Max angle in degrees
  float min_angle = 0;
  float max_angle = 180;

  ServoData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string>& unused_keys);

  bool check();

  static std::string get_device_class() { return "servo"; }
};