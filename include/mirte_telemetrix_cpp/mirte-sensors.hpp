#pragma once
#include "mirte_telemetrix_cpp/mirte-board.hpp"
#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/srv/get_pin_value.hpp>

class Mirte_Sensors {
public:
  Mirte_Sensors(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                std::shared_ptr<Mirte_Board> board,
                std::shared_ptr<Parser> parser);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<Mirte_Sensor>> sensors;
  rclcpp::TimerBase::SharedPtr timer;
  void publish();
  void stop();

  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetPinValue>> pin_service;
  enum class PIN_USE { DIGITAL_IN, DIGITAL_OUT, ANALOG_IN, ANALOG_OUT, UNUSED };
  std::map<pin_t, std::tuple<PIN_USE, int, bool, bool>>
      pin_map; // pin -> (is_digital, value,analog_cb, digital_cb )
  bool
  pin_callback(const std::shared_ptr<mirte_msgs::srv::GetPinValue::Request> req,
               std::shared_ptr<mirte_msgs::srv::GetPinValue::Response> res);
};
