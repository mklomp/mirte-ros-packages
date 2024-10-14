#pragma once
#include <memory>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/modules/hiwonder/hiwonder_servo.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/hiwonder_data.hpp>

#include <std_srvs/srv/set_bool.hpp>

class HiWonderBus_module : public Mirte_module
{
public:
  HiWonderBus_module(
    NodeData node_data, HiWonderBusData bus_data, std::shared_ptr<tmx_cpp::Modules> modules);

  HiWonderBusData data;
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus;
  std::vector<std::shared_ptr<Hiwonder_servo>> servos;

  static std::vector<std::shared_ptr<HiWonderBus_module>> get_hiwonder_modules(
    NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules);

private:
  void position_cb(std::vector<tmx_cpp::HiwonderServo_module::Servo_pos>);
  void verify_cb(int, bool);
  void range_cb(int, uint16_t, uint16_t);
  void offset_cb(int, uint16_t);

  // ROS Enable Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_all_servos_service;
  bool enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);
};
