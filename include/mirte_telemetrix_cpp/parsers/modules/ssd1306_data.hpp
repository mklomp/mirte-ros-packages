#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

class SSD1306Data : public I2CModuleData
{
public:
  uint8_t width = 128;  // Hardcoded in the Pico
  uint8_t height = 64;  // Hardcoded in the Pico

  SSD1306Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys);

  bool check();

  static std::string get_module_type() { return "ssd1306"; };
};