#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

#include <std_msgs/msg/header.hpp>

// namespace mirte_telemetrix_cpp {

class Mirte_Sensor
{
public:
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<uint8_t> pins;

  virtual void publish() = 0;
  std::string name;
  std::string frame_id = "";
  virtual void stop() {}
  auto get_header()
  {
    std_msgs::msg::Header header;
    header.stamp = nh->now();
    header.frame_id = frame_id;

    return header;
  }
  Mirte_Sensor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, SensorData data);

  Mirte_Sensor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
};

// }  // namespace mirte_telemetrix_cpp