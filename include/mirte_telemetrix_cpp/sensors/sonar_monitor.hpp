#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>
#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/srv/get_distance.hpp>
#include <sensor_msgs/msg/range.hpp>

class SonarMonitor : public Mirte_Sensor
{
public:
  SonarMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
    SonarData sonar_data);

  void publish();

  SonarData sonar_data;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> sonar_pub;
  static std::vector<std::shared_ptr<SonarMonitor>> get_sonar_monitors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Parser> parser);
  void callback(uint16_t value);
  uint16_t value;
  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetDistance>> sonar_service;
  bool service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetDistance::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetDistance::Response> res);
};
