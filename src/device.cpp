#include <functional>

#include <mirte_telemetrix_cpp/device.hpp>

TelemetrixDevice::TelemetrixDevice(NodeData node_data, std::vector<uint8_t> pins, DeviceData data)
: TelemetrixDevice(node_data, pins, data, rclcpp::CallbackGroupType::MutuallyExclusive)
{
}

TelemetrixDevice::TelemetrixDevice(
  NodeData node_data, std::vector<uint8_t> pins, DeviceData data,
  rclcpp::CallbackGroupType callback_group_type)
: tmx(node_data.tmx),
  nh(node_data.nh),
  board(node_data.board),
  pins(pins),
  logger(nh->get_logger()),
  name(data.name),
  frame_id(data.frame_id)
{
  this->callback_group = nh->create_callback_group(callback_group_type);

  // TODO: Starting from Jazzy Set auto_start false, so they can be started at the end of the initialization.
  this->device_timer = nh->create_wall_timer(
    data.duration, [this]() { this->device_timer_callback(); }, this->callback_group);
}
