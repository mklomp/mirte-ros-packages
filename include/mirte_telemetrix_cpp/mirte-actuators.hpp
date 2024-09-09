#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mirte_msgs/srv/set_pin_value.hpp>

#include "mirte_telemetrix_cpp/parsers/actuators.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <string>
#include <tmx_cpp/tmx.hpp>
#include <vector>

class Mirte_Actuator;
class Mirte_Actuators {
public:
  Mirte_Actuators(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                  std::shared_ptr<Mirte_Board> board,
                  std::shared_ptr<Parser> parser);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<Mirte_Actuator>> actuators;

  rclcpp::Service<mirte_msgs::srv::SetPinValue>::SharedPtr set_pin_value_service;

private:
  void set_pin_value_service_callback(const mirte_msgs::srv::SetPinValue::Request::ConstSharedPtr req,
                                      mirte_msgs::srv::SetPinValue::Response::SharedPtr res);
};

class Mirte_Actuator {
public:
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<pin_t> pins;
  std::string name;
  auto get_header() {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Actuator(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                 std::shared_ptr<Mirte_Board> board, std::vector<pin_t> pins,
                 std::string name);
  virtual ~Mirte_Actuator() {}
};

#include <mirte_telemetrix_cpp/actuators/servo.hpp>
#include <mirte_telemetrix_cpp/actuators/motor.hpp>
