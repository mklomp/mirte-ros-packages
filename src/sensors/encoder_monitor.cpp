#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>

#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/srv/get_encoder.hpp>

EncoderMonitor::EncoderMonitor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  EncoderData encoder_data)
: Mirte_Sensor(nh, tmx, board, {encoder_data.pinA, encoder_data.pinB}, (SensorData)encoder_data),
  encoder_data(encoder_data)
{
  encoder_pub = nh->create_publisher<mirte_msgs::msg::Encoder>("encoder/" + encoder_data.name, 1);
  encoder_service = nh->create_service<mirte_msgs::srv::GetEncoder>(
    "get_encoder_" + encoder_data.name,
    std::bind(
      &EncoderMonitor::service_callback, this, std::placeholders::_1, std::placeholders::_2));

  tmx->attach_encoder(
    encoder_data.pinA, encoder_data.pinB, [this](auto pin, auto value) { this->callback(value); });
}

void EncoderMonitor::callback(int16_t value)
{
  value += value;
  publish();
}

void EncoderMonitor::publish()
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
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<EncoderMonitor>> sensors;
  auto encoders = parse_all<EncoderData>(parser, board);
  for (auto encoder : encoders) {
    sensors.push_back(std::make_shared<EncoderMonitor>(nh, tmx, board, encoder));
    std::cout << "Add Encoder: " << encoder.name << std::endl;
  }
  return sensors;
}
