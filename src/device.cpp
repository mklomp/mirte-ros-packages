#include <mirte_telemetrix_cpp/device.hpp>

TelemetrixDevice::TelemetrixDevice(NodeData node_data, std::vector<uint8_t> pins, DeviceData data)
: tmx(node_data.tmx),
  nh(node_data.nh),
  board(node_data.board),
  pins(pins),
  name(data.name),
  frame_id(data.frame_id)
{
}