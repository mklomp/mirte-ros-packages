#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>

#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/srv/get_encoder.hpp>

EncoderMonitor::EncoderMonitor(std::shared_ptr<rclcpp::Node> nh,
                               std::shared_ptr<TMX> tmx,
                               std::shared_ptr<Mirte_Board> board,
                               std::shared_ptr<Encoder_data> encoder_data)
    : Mirte_Sensor(nh, tmx, board, {encoder_data->pinA, encoder_data->pinB},
                   encoder_data->name) {
  this->encoder_data = encoder_data;
  encoder_pub = nh->create_publisher<mirte_msgs::msg::Encoder>(
      "encoder/" + encoder_data->name, 1);
  this->encoder_service = nh->create_service<mirte_msgs::srv::GetEncoder>(
      "get_encoder_" + encoder_data->name,
      std::bind(&EncoderMonitor::service_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  tmx->attach_encoder(encoder_data->pinA, encoder_data->pinB,
                      [this](auto pin, auto value) { this->callback(value); });
}

void EncoderMonitor::callback(int16_t value) {
  this->value += value;
  this->publish();
}

void EncoderMonitor::publish() {
  mirte_msgs::msg::Encoder msg;
  msg.value = this->value;
  this->encoder_pub->publish(msg);
}

bool EncoderMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetEncoder::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetEncoder::Response> res) {
  res->data = this->value;
  return true;
}

std::vector<std::shared_ptr<EncoderMonitor>>
EncoderMonitor::get_encoder_monitors(std::shared_ptr<rclcpp::Node> nh,
                                     std::shared_ptr<TMX> tmx,
                                     std::shared_ptr<Mirte_Board> board,
                                     std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<EncoderMonitor>> sensors;
  auto encoders = Encoder_data::parse_encoder_data(parser, board);
  for (auto encoder : encoders) {
    sensors.push_back(
        std::make_shared<EncoderMonitor>(nh, tmx, board, encoder));
    std::cout << "add encoder " << encoder->name << std::endl;
  }
  return sensors;
}
