#include <memory>
#include <vector>

#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>

#include <mirte_telemetrix_cpp/sensors/sonar_monitor.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <mirte_msgs/srv/get_range.hpp>

std::vector<std::shared_ptr<SonarMonitor>> SonarMonitor::get_sonar_monitors(
  NodeData node_data, std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<SonarMonitor>> sensors;
  auto sonars = parse_all<SonarData>(parser, node_data.board);
  for (auto sonar : sonars) {
    sensors.push_back(std::make_shared<SonarMonitor>(node_data, sonar));
    // std::cout << "Add Sonar: " << sonar.name << std::endl;
  }
  return sensors;
}

SonarMonitor::SonarMonitor(NodeData node_data, SonarData sonar_data)
: Mirte_Sensor(node_data, {sonar_data.trigger, sonar_data.echo}, (SensorData)sonar_data),
  sonar_data(sonar_data)
{
  this->logger = this->logger.get_child(sonar_data.get_device_class()).get_child(sonar_data.name);

  // Use default QOS for sensor publishers as specified in REP2003
  sonar_pub = nh->create_publisher<sensor_msgs::msg::Range>(
    "distance/" + sonar_data.name, rclcpp::SystemDefaultsQoS());

  sonar_service = nh->create_service<mirte_msgs::srv::GetRange>(
    "distance/" + sonar_data.name + "/get_range",
    std::bind(&SonarMonitor::service_callback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->attach_sonar(
    sonar_data.trigger, sonar_data.echo, [this](auto pin, auto value) { this->callback(value); });
}

void SonarMonitor::callback(uint16_t value)
{
  // Report Errors as specified in REP0117
  if (value == 0xFFFF) {
    // Should not occure
    this->distance = NAN;
    RCLCPP_DEBUG(logger, "Some weird error which shouldn't occure or no new data was generated?");
  } else if (value == 0xFFFE) {
    // Too long since trigger, resulting in invalid reading
    this->distance = NAN;
    RCLCPP_DEBUG(logger, "Too long since trigger");
  } else if (value == 0xFFFD) {
    // Timeout, so detection is out of range
    this->distance = INFINITY;
    RCLCPP_DEBUG(logger, "Object outside of range");
  } else if (value == 0xFFFC) {
    this->distance = NAN;
    RCLCPP_DEBUG(logger, "No new distance measurement was created in time");
  } else {
    // The reading is possibly valid.
    auto raw_distance = value / 100.0;
    RCLCPP_DEBUG(logger, "%d", value);

    if (raw_distance < min_range)
      this->distance = -INFINITY;
    else if (raw_distance > max_range)
      this->distance = INFINITY;
    else
      this->distance = raw_distance;
  }
  this->update();
  this->device_timer->reset();
}

void SonarMonitor::update()
{
  sensor_msgs::msg::Range msg;
  msg.header = this->get_header();
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = M_PI / 12.0;  // 15 degrees, according to the HC-SR04 datasheet
  msg.min_range = min_range;
  msg.max_range = max_range;
  msg.range = this->distance;
  this->range = msg;
  this->sonar_pub->publish(msg);
}

bool SonarMonitor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::GetRange::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetRange::Response> res)
{
  res->range = this->range;
  return true;
}
