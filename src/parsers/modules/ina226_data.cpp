#include <mirte_telemetrix_cpp/parsers/modules/ina226_data.hpp>

INA226Data::INA226Data(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: I2CModuleData(parser, board, name, parameters)
{
  // TODO: Temporary new default for address
  if ((!parameters.count("id")) && this->addr == 0xFF) this->addr = 0x40;

  if (parameters.count("max_current")) this->max_current = get_float(parameters["max_current"]);

  if (parameters.count("max_voltage")) this->max_voltage = get_float(parameters["max_voltage"]);

  if (parameters.count("min_voltage")) this->min_voltage = get_float(parameters["min_voltage"]);

  if (parameters.count("power_low_time"))
    this->power_low_time = get_float(parameters["power_low_time"]);  //.get<float>();
}

bool INA226Data::check() { return power_low_time > 0 && I2CModuleData::check(get_module_type()); }