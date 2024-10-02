#pragma once

#include <memory>

#include <tmx_cpp/sensors/MPU9250.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/mpu9250_data.hpp>

#include <mirte_msgs/srv/get_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>

class MPU9250_sensor : public Mirte_module
{
public:
  MPU9250_sensor(
    NodeData node_data, MPU9250Data imu_data, std::shared_ptr<tmx_cpp::Sensors> sensors);

  MPU9250Data data;
  std::shared_ptr<tmx_cpp::MPU9250_module> mpu9250;

  void data_cb(
    std::vector<float> acceleration, std::vector<float> gyro, std::vector<float> magnetic_field,
    std::vector<float> quaternion);

  static std::vector<std::shared_ptr<MPU9250_sensor>> get_mpu_modules(
    NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Sensors> sensors);

private:
  sensor_msgs::msg::Imu msg;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

  rclcpp::Service<mirte_msgs::srv::GetImu>::SharedPtr imu_service;

  void get_imu_service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetImu::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetImu::Response> res);
};
