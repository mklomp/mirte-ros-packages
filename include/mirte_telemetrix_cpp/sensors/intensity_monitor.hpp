#pragma once
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/intensity_data.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>
#include <mirte_msgs/srv/get_intensity.hpp>
#include <mirte_msgs/srv/get_intensity_digital.hpp>

class IntensityMonitor : public Mirte_Sensor {
  public:
    IntensityMonitor(NodeData node_data, std::vector<pin_t> pins, IntensityData intensity_data);

    virtual void update() override = 0;

    static std::vector<std::shared_ptr<IntensityMonitor>> get_intensity_monitors(
      NodeData node_data, std::shared_ptr<Parser> parser);

    IntensityData intensity_data;
};

class DigitalIntensityMonitor : public IntensityMonitor {
  public:
    DigitalIntensityMonitor(NodeData node_data, IntensityData intensity_data);
    virtual void update() override;

    void callback(uint16_t value);
    std::atomic<bool> value;

    rclcpp::Publisher<mirte_msgs::msg::IntensityDigital>::SharedPtr intensity_pub;
    rclcpp::Service<mirte_msgs::srv::GetIntensityDigital>::SharedPtr intensity_service;

    bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res);
};

class AnalogIntensityMonitor : public IntensityMonitor {
  public:
    AnalogIntensityMonitor(NodeData node_data, IntensityData intensity_data);
    virtual void update() override;

    void callback(uint16_t value);
    std::atomic<uint16_t> value;

    rclcpp::Publisher<mirte_msgs::msg::Intensity>::SharedPtr intensity_pub;
    rclcpp::Service<mirte_msgs::srv::GetIntensity>::SharedPtr intensity_service;

    bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res);
};
