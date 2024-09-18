#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/parsers.hpp>
#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

SensorData::SensorData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::string sensor_type, std::map<std::string, rclcpp::ParameterValue> parameters)
: DeviceData(parser, board, name, sensor_type, parameters)
{
}

SensorData::SensorData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: SensorData(parser, board, name, this->get_device_class(), parameters)
{
}