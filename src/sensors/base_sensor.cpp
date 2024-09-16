#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

Mirte_Sensor::Mirte_Sensor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, std::string name)
: nh(nh), tmx(tmx), board(board), pins(pins), name(name)
{
}

Mirte_Sensor::Mirte_Sensor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, SensorData data)
: nh(nh), tmx(tmx), board(board), pins(pins), name(data.name), frame_id(data.frame_id)
{
}
