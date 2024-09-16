#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_telemetrix_cpp/parsers/p_sensors.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>
#include <mirte_msgs/srv/get_intensity.hpp>
#include <mirte_msgs/srv/get_intensity_digital.hpp>

class IntensityMonitor : public Mirte_Sensor {
public:
  IntensityMonitor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                   std::shared_ptr<Mirte_Board> board, std::vector<pin_t> pins,
                   std::string name)
      : Mirte_Sensor(nh, tmx, board, pins, name) {}
  virtual void publish() = 0;
  static std::vector<std::shared_ptr<IntensityMonitor>> get_intensity_monitors(
      std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
      std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser);
  ~IntensityMonitor() {}
  std::shared_ptr<Intensity_data> intensity_data;
};

class Digital_IntensityMonitor : public IntensityMonitor {
public:
  Digital_IntensityMonitor(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::shared_ptr<Intensity_data> intensity_data);
  void publish();
  void callback(uint16_t value);
  bool value;
  
  rclcpp::Publisher<mirte_msgs::msg::IntensityDigital>::SharedPtr intensity_pub;
  rclcpp::Service<mirte_msgs::srv::GetIntensityDigital>::SharedPtr intensity_service;

  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res);
};

class Analog_IntensityMonitor : public IntensityMonitor {
public:
  Analog_IntensityMonitor(std::shared_ptr<rclcpp::Node> nh,
                          std::shared_ptr<TMX> tmx,
                          std::shared_ptr<Mirte_Board> board,
                          std::shared_ptr<Intensity_data> intensity_data);
  void publish();
  void callback(uint16_t value);
  uint16_t value;
  
  rclcpp::Publisher<mirte_msgs::msg::Intensity>::SharedPtr intensity_pub;
  rclcpp::Service<mirte_msgs::srv::GetIntensity>::SharedPtr intensity_service;

  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res);
};
