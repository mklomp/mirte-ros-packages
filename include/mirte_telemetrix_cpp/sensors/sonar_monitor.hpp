#pragma once
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <mirte_msgs/srv/get_range.hpp>

class SonarMonitor : public Mirte_Sensor {
  public:
    /// @brief The minimum range in meters. 0.02 Meters for the HC-SR04.
    const double min_range = 0.02;
    /// @brief The maximum range in meters. 4.50 Meters for the HC-SR04.
    const double max_range = 4.5;

    SonarMonitor(NodeData node_data, SonarData sonar_data);

    virtual void update() override;

    SonarData sonar_data;
    static std::vector<std::shared_ptr<SonarMonitor>> get_sonar_monitors(
      NodeData node_data, std::shared_ptr<Parser> parser);
    void callback(uint16_t value);

  private:
    /// @brief The last recorded distance.
    std::atomic<double> distance = NAN;
    std::mutex msg_mutex;
    sensor_msgs::msg::Range range;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub;

    rclcpp::Service<mirte_msgs::srv::GetRange>::SharedPtr sonar_service;
    bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetRange::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetRange::Response> res);
};
