#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>

IntensityMonitor::IntensityMonitor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::vector<pin_t> pins, IntensityData intensity_data)
: Mirte_Sensor(nh, tmx, board, pins, (SensorData)intensity_data), intensity_data(intensity_data)
{
}

std::vector<std::shared_ptr<IntensityMonitor>> IntensityMonitor::get_intensity_monitors(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;
  auto ir_sensors = parse_all<IntensityData>(parser, board);
  for (auto ir_data : ir_sensors) {
    if (ir_data.a_pin != (pin_t)-1) {
      sensors.push_back(std::make_shared<AnalogIntensityMonitor>(nh, tmx, board, ir_data));
      std::cout << "Add Analog Intensity: " << ir_data.name << std::endl;
    }
    if (ir_data.d_pin != (pin_t)-1) {
      sensors.push_back(std::make_shared<DigitalIntensityMonitor>(nh, tmx, board, ir_data));
      std::cout << "Add Digital Intensity: " << ir_data.name << std::endl;
    }
  }
  return sensors;
}

void DigitalIntensityMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}

void DigitalIntensityMonitor::publish()
{
  mirte_msgs::msg::IntensityDigital msg;

  msg.header = get_header();
  msg.value = value;

  intensity_pub->publish(msg);
}

DigitalIntensityMonitor::DigitalIntensityMonitor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
  IntensityData intensity_data)
: IntensityMonitor(nh, tmx, board, {intensity_data.d_pin}, intensity_data)
{
  intensity_pub = nh->create_publisher<mirte_msgs::msg::IntensityDigital>(
    "intensity/" + intensity_data.name + "_digital", 1);

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensityDigital>(
    "get_intensity_" + intensity_data.name + "_digital",
    std::bind(
      &DigitalIntensityMonitor::service_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  tmx->setPinMode(intensity_data.d_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT, true, 0);
  tmx->add_digital_callback(
    intensity_data.d_pin, [this](auto pin, auto value) { this->callback(value); });
}

AnalogIntensityMonitor::AnalogIntensityMonitor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
  IntensityData intensity_data)
: IntensityMonitor(nh, tmx, board, {intensity_data.a_pin}, intensity_data)
{
  intensity_pub =
    nh->create_publisher<mirte_msgs::msg::Intensity>("intensity/" + intensity_data.name, 1);

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensity>(
    "get_intensity_" + intensity_data.name, std::bind(
                                              &AnalogIntensityMonitor::service_callback, this,
                                              std::placeholders::_1, std::placeholders::_2));

  tmx->setPinMode(intensity_data.a_pin, tmx_cpp::TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->add_analog_callback(
    intensity_data.a_pin, [this](auto pin, auto value) { this->callback(value); });
}

void AnalogIntensityMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}

void AnalogIntensityMonitor::publish()
{
  mirte_msgs::msg::Intensity msg;

  msg.header = get_header();
  msg.value = value;

  intensity_pub->publish(msg);
}

bool DigitalIntensityMonitor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res)
{
  res->data = value;
  return true;
}

bool AnalogIntensityMonitor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res)
{
  res->data = value;
  return true;
}
