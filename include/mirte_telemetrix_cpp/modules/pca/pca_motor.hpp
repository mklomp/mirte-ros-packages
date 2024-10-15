#pragma once
#include <memory>
#include <tuple>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_motor_data.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <std_msgs/msg/int32.hpp>

class PCA_Motor
{
public:
  PCA_Motor(
    NodeData node_data, std::shared_ptr<PCA_Motor_data> motor_data,
    std::shared_ptr<tmx_cpp::PCA9685_module> pca9685);
  std::shared_ptr<PCA_Motor_data> motor_data;
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685_mod;

  std::tuple<uint32_t, uint32_t> calc_pwm_speed(int speed);

  std::vector<tmx_cpp::PCA9685_module::PWM_val> get_multi_speed_pwm(int speed);
  void set_speed(int speed);

  // Stolen from mirtes-actuators.hpp::motor, but it was too shit to inherit
  // from that one as well.

  //   void set_speed_old(
  //     int speed, bool direct = true,
  //     std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals = {});

private:
  int last_speed = 0;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ros_client;
  void motor_callback(const std_msgs::msg::Int32 & msg);

  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;

  void set_speed_service_callback(
    const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res);
};