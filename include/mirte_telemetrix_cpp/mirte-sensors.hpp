#pragma once
#include <tmx_cpp/tmx.hpp>

#include "mirte_telemetrix_cpp/mirte-board.hpp"
#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/srv/get_pin_value.hpp>

class Mirte_Sensors {
  public:
    Mirte_Sensors(NodeData node_data, std::shared_ptr<Parser> parser);
    std::shared_ptr<tmx_cpp::TMX> tmx;
    std::shared_ptr<rclcpp::Node> nh;
    std::shared_ptr<Mirte_Board> board;
    std::vector<std::shared_ptr<Mirte_Sensor>> sensors;

    void stop();

    enum class PIN_USE { DIGITAL_IN, DIGITAL_OUT, ANALOG_IN, ANALOG_OUT, UNUSED };
    std::map<pin_t, std::tuple<PIN_USE, int, bool, bool>>
      pin_map;  // pin -> (is_digital, value,analog_cb, digital_cb )

  private:
    // Service: get_pin_value
    rclcpp::Service<mirte_msgs::srv::GetPinValue>::SharedPtr pin_service;

    void pin_callback(
      const mirte_msgs::srv::GetPinValue::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetPinValue::Response::SharedPtr res);
};
