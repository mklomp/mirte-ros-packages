#pragma once
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/parsers/p_modules.hpp>

#include <mirte_msgs/msg/servo_position.hpp>
#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Hiwonder_servo;
class Hiwonder_bus_module : public Mirte_module
{
public:
  Hiwonder_bus_module(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::string name, std::shared_ptr<tmx_cpp::Modules> modules,
    std::shared_ptr<Hiwonder_bus_data> bus_data);
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus;
  std::vector<std::shared_ptr<Hiwonder_servo>> servos;
  //   std::vector<std::shared_ptr<PCA_Servo>> servos;

  std::shared_ptr<Hiwonder_bus_data> bus_data;
  static std::vector<std::shared_ptr<Hiwonder_bus_module>>

  get_hiwonder_modules(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser,
    std::shared_ptr<tmx_cpp::Modules> modules);

  void position_cb(std::vector<tmx_cpp::HiwonderServo_module::Servo_pos>);
  void verify_cb(int, bool);
  void range_cb(int, uint16_t, uint16_t);
  void offset_cb(int, uint16_t);

  // ROS:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
  bool enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);
};

class Hiwonder_servo
{
public:
  Hiwonder_servo(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Hiwonder_servo_data> servo_data,
    std::shared_ptr<tmx_cpp::HiwonderServo_module> bus);
  std::shared_ptr<Hiwonder_servo_data> servo_data;
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod;

  // callbacks from the pico
  void position_cb(tmx_cpp::HiwonderServo_module::Servo_pos & pos);

  // ROS:
  //  set_x_servo_enable
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
  bool enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // set_x_servo_angle
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr angle_service;
  bool angle_cb(
    const std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetServoAngle::Response> res);

  // get_x_servo_range
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr range_service;
  bool range_cb(
    const std::shared_ptr<mirte_msgs::srv::GetServoRange::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetServoRange::Response> res);

  // /servos/x/position publisher
  rclcpp::Publisher<mirte_msgs::msg::ServoPosition>::SharedPtr position_pub;

  // angle calcs
  uint16_t calc_angle_out(float angle_in);
  float calc_angle_in(uint16_t angle_out);
};