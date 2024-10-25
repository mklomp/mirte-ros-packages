#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mirte_msgs/msg/encoder.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/encoder_data.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/srv/get_encoder.hpp>

class EncoderMonitor : public Mirte_Sensor {
  public:
    EncoderMonitor(NodeData node_data, EncoderData encoder_data);

    void update();

    EncoderData encoder_data;
    std::shared_ptr<rclcpp::Publisher<mirte_msgs::msg::Encoder>> encoder_pub;
    static std::vector<std::shared_ptr<EncoderMonitor>> get_encoder_monitors(
      NodeData node_data, std::shared_ptr<Parser> parser);
    void callback(int16_t value);
    int16_t value = 0;
    std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetEncoder>> encoder_service;
    bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetEncoder::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetEncoder::Response> res);
    ~EncoderMonitor(){};
};