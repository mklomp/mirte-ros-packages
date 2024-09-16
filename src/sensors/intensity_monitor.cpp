#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>

std::vector<std::shared_ptr<IntensityMonitor>>
IntensityMonitor::get_intensity_monitors(std::shared_ptr<rclcpp::Node> nh,
                                         std::shared_ptr<TMX> tmx,
                                         std::shared_ptr<Mirte_Board> board,
                                         std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;
  auto irs = Intensity_data::parse_intensity_data(parser, board);
  for (auto ir : irs) {
    if (ir->a_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<Analog_IntensityMonitor>(nh, tmx, board, ir));
      std::cout << "add analog intensity" << ir->name << std::endl;
    }
    if (ir->d_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<Digital_IntensityMonitor>(nh, tmx, board, ir));
      std::cout << "add digital intensity" << ir->name << std::endl;
    }
  }
  return sensors;
}

void Digital_IntensityMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}

void Digital_IntensityMonitor::publish() {
  mirte_msgs::msg::IntensityDigital msg;
  msg.value = this->value;
  this->intensity_pub->publish(msg);
}

Digital_IntensityMonitor::Digital_IntensityMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Intensity_data> intensity_data)
    : IntensityMonitor(nh, tmx, board, {intensity_data->d_pin},
                       intensity_data->name) {
  this->intensity_data = intensity_data;
  intensity_pub = nh->create_publisher<mirte_msgs::msg::IntensityDigital>(
      "intensity/" + intensity_data->name + "_digital", 1);

  this->intensity_service =
      nh->create_service<mirte_msgs::srv::GetIntensityDigital>(
          "get_intensity_" + intensity_data->name + "_digital",
          std::bind(&Digital_IntensityMonitor::service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
  tmx->setPinMode(intensity_data->d_pin, TMX::PIN_MODES::DIGITAL_INPUT, true,
                  0);
  tmx->add_digital_callback(
      intensity_data->d_pin,
      [this](auto pin, auto value) { this->callback(value); });
}

Analog_IntensityMonitor::Analog_IntensityMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Intensity_data> intensity_data)
    : IntensityMonitor(nh, tmx, board, {intensity_data->a_pin},
                       intensity_data->name) {
  this->intensity_data = intensity_data;
  intensity_pub = nh->create_publisher<mirte_msgs::msg::Intensity>(
      "intensity/" + intensity_data->name, 1);

  this->intensity_service = nh->create_service<mirte_msgs::srv::GetIntensity>(
      "get_intensity_" + intensity_data->name,
      std::bind(&Analog_IntensityMonitor::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  tmx->setPinMode(intensity_data->a_pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->add_analog_callback(intensity_data->a_pin, [this](auto pin, auto value) {
    this->callback(value);
  });
}

void Analog_IntensityMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}

void Analog_IntensityMonitor::publish() {
  mirte_msgs::msg::Intensity msg;
  msg.value = this->value;
  this->intensity_pub->publish(msg);
}

bool Digital_IntensityMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res) {
  res->data = this->value;
  return true;
}

bool Analog_IntensityMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res) {
  res->data = this->value;
  return true;
}
