#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

class INA226Data : public I2CModuleData
{
public:
  float max_current = 10;
  float max_voltage = 14;
  float min_voltage = 10.5;
  float power_low_time = 5;

  INA226Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters);

  bool check();

  static std::string get_module_type() { return "ina226"; };
};