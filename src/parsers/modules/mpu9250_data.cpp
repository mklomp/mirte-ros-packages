#include <mirte_telemetrix_cpp/parsers/modules/mpu9250_data.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/parameter_value.hpp>
#include "mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp"


// Use the default frame_id `imu_link` as specified in REP0145
MPU9250Data::MPU9250Data(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: I2CModuleData(parser, board, name, insert_default_frame_id(parameters), insert_default_frame_id(unused_keys))
{
  // Set default for address
  // FIXME: Currently fixed in Telemetrix Firmware
  if ((!parameters.count("addr")) && this->addr == 0xFF) this->addr = 0x68;
}

std::map<std::string, rclcpp::ParameterValue> MPU9250Data::insert_default_frame_id(std::map<std::string, rclcpp::ParameterValue> parameters) {
  parameters.emplace("frame_id", rclcpp::ParameterValue("imu_link"));
  return parameters;
}


std::set<std::string> & MPU9250Data::insert_default_frame_id(std::set<std::string> & unused_keys) {
  unused_keys.emplace("frame_id");
  return unused_keys;
}

bool MPU9250Data::check() { return I2CModuleData::check(get_module_type()); }
