#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>
#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/srv/get_distance.hpp>
#include <sensor_msgs/msg/range.hpp>

class SonarMonitor : public Mirte_Sensor
{
public:
  /// @brief The minimum range in meters. 0.02 Meters for the HC-SR04.
  const double min_range = 0.02;
  /// @brief The maximum range in meters. 4.50 Meters for the HC-SR04.
  const double max_range = 4.5;

  SonarMonitor(NodeData node_data, SonarData sonar_data);

  void update();

  SonarData sonar_data;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> sonar_pub;
  static std::vector<std::shared_ptr<SonarMonitor>> get_sonar_monitors(
    NodeData node_data, std::shared_ptr<Parser> parser);
  void callback(uint16_t value);

private:
  /// @brief The last recorded distance.
  double distance = NAN;

  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetDistance>> sonar_service;
  bool service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetDistance::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetDistance::Response> res);
};
