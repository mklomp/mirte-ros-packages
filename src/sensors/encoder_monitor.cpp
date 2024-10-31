#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>

#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/srv/get_encoder.hpp>

EncoderMonitor::EncoderMonitor(NodeData node_data, EncoderData encoder_data)
: Mirte_Sensor(node_data, {encoder_data.pinA, encoder_data.pinB}, (SensorData)encoder_data),
  encoder_data(encoder_data)
{
  // Use default QOS for sensor publishers as specified in REP2003
  encoder_pub = nh->create_publisher<mirte_msgs::msg::Encoder>(
    "encoder/" + encoder_data.name, rclcpp::SystemDefaultsQoS());

  encoder_service = nh->create_service<mirte_msgs::srv::GetEncoder>(
    "encoder/" + encoder_data.name + "/get_encoder",
    std::bind(
      &EncoderMonitor::service_callback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->attach_encoder(
    encoder_data.pinA, encoder_data.pinB, [this](auto pin, auto value) { this->callback(value); });
}

void EncoderMonitor::callback(int16_t value)
{
  this->value += value;
  update();
  this->device_timer->reset();
}

void EncoderMonitor::update()
{
  mirte_msgs::msg::Encoder msg;

  msg.header = get_header();
  msg.value = value;

  encoder_pub->publish(msg);
}

bool EncoderMonitor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::GetEncoder::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetEncoder::Response> res)
{
  res->data = value;
  return true;
}

std::vector<std::shared_ptr<EncoderMonitor>> EncoderMonitor::get_encoder_monitors(
  NodeData node_data, std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<EncoderMonitor>> sensors;
  auto encoders = parse_all<EncoderData>(parser, node_data.board);
  for (auto encoder : encoders) {
    sensors.push_back(std::make_shared<EncoderMonitor>(node_data, encoder));
    // std::cout << "Add Encoder: " << encoder.name << std::endl;
  }
  return sensors;
}
