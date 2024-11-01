#pragma once
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mirte_msgs/srv/set_pin_value.hpp>

class Mirte_Actuators {
  public:
    Mirte_Actuators(NodeData data, std::shared_ptr<Parser> parser);

    std::shared_ptr<tmx_cpp::TMX> tmx;
    std::shared_ptr<rclcpp::Node> nh;
    std::shared_ptr<Mirte_Board> board;
    std::vector<std::shared_ptr<TelemetrixDevice>> actuators;

  private:
    // Service: set_pin_value
    rclcpp::Service<mirte_msgs::srv::SetPinValue>::SharedPtr set_pin_value_service;

    void set_pin_value_service_callback(
      const mirte_msgs::srv::SetPinValue::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetPinValue::Response::SharedPtr res);
};
