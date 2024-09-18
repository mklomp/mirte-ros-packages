#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

Mirte_Sensor::Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins, SensorData data)
: nh(node_data.nh),
  tmx(node_data.tmx),
  board(node_data.board),
  pins(pins),
  name(data.name),
  frame_id(data.frame_id)
{
}