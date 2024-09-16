#include <memory>
#include <vector>

#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>
#include <mirte_telemetrix_cpp/sensors/sonar_monitor.hpp>

#include <mirte_msgs/srv/get_distance.hpp>
#include <sensor_msgs/msg/range.hpp>

std::vector<std::shared_ptr<SonarMonitor>> SonarMonitor::get_sonar_monitors(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<SonarMonitor>> sensors;
  auto sonars = parse_all<SonarData>(parser, board);
  for (auto sonar : sonars) {
    sensors.push_back(std::make_shared<SonarMonitor>(nh, tmx, board, sonar));
    std::cout << "Add Sonar: " << sonar.name << std::endl;
  }
  return sensors;
}

SonarMonitor::SonarMonitor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  SonarData sonar_data)
: Mirte_Sensor(nh, tmx, board, {sonar_data.trigger, sonar_data.echo}, (SensorData)sonar_data),
  sonar_data(sonar_data)
{
  sonar_pub = nh->create_publisher<sensor_msgs::msg::Range>("distance/" + sonar_data.name, 1);

  sonar_service = nh->create_service<mirte_msgs::srv::GetDistance>(
    "get_distance_" + sonar_data.name,
    std::bind(&SonarMonitor::service_callback, this, std::placeholders::_1, std::placeholders::_2));

  tmx->attach_sonar(
    sonar_data.trigger, sonar_data.echo, [this](auto pin, auto value) { this->callback(value); });
}

void SonarMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}

void SonarMonitor::publish()
{
  sensor_msgs::msg::Range msg;
  msg.header = this->get_header();
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = M_PI / 12.0;  // 15 degrees, according to the HC-SR04 datasheet
  msg.min_range = 0.02;
  msg.max_range = 4.5;
  msg.range = this->value / 100.0;
  this->sonar_pub->publish(msg);
}

bool SonarMonitor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::GetDistance::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetDistance::Response> res)
{
  res->data = this->value / 1000.0;
  return true;
}