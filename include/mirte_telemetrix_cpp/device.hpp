#pragma once
#include <memory>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

#include <std_msgs/msg/header.hpp>

// namespace mirte_telemetrix_cpp {

class TelemetrixDevice {
  public:
    std::shared_ptr<tmx_cpp::TMX> tmx;
    std::shared_ptr<rclcpp::Node> nh;
    std::shared_ptr<Mirte_Board> board;
    std::vector<uint8_t> pins;
    rclcpp::Logger logger;

    virtual void update(){};
    std::string name;
    std::string frame_id = "";

    rclcpp::CallbackGroup::SharedPtr callback_group;

    virtual void stop() {}
    auto get_header()
    {
      std_msgs::msg::Header header;
      header.stamp = nh->now();
      header.frame_id = frame_id;

      return header;
    }
    TelemetrixDevice(NodeData node_data, std::vector<uint8_t> pins, DeviceData data);
    TelemetrixDevice(
      NodeData node_data, std::vector<uint8_t> pins, DeviceData data,
      rclcpp::CallbackGroupType callback_group_type);

    virtual void device_timer_callback(){};

  protected:
    rclcpp::TimerBase::SharedPtr device_timer;
};

// }  // namespace mirte_telemetrix_cpp
