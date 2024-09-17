#pragma once
#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/parsers/p_modules.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <mirte_msgs/srv/set_speed_multiple.hpp>
#include <std_msgs/msg/int32.hpp>

class PCA_Motor;
class PCA_Module : public Mirte_module
{
public:
  PCA_Module(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::string name, std::shared_ptr<tmx_cpp::Modules> modules,
    std::shared_ptr<PCA_data> pca_data);
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685;
  std::vector<std::shared_ptr<PCA_Motor>> motors;

  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::SetSpeedMultiple>> motor_service;
  bool motor_service_cb(
    const std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Response> res);
  //   std::vector<std::shared_ptr<PCA_Servo>> servos;
  static std::vector<std::shared_ptr<PCA_Module>> get_pca_modules(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser,
    std::shared_ptr<tmx_cpp::Modules> modules);
};

class PCA_Motor
{
public:
  PCA_Motor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<PCA_Motor_data> motor_data,
    std::shared_ptr<tmx_cpp::PCA9685_module> pca9685);
  std::shared_ptr<PCA_Motor_data> motor_data;
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685_mod;
  // Stolen from mirtes-actuators.hpp::motor, but it was too shit to inherit
  // from that one as well.
  void set_speed(
    int speed, bool direct = true,
    std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals = {});
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr motor_service;
  bool service_callback(
    const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res);
  void motor_callback(const std_msgs::msg::Int32 & msg);
  int last_speed = 0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ros_client;
};