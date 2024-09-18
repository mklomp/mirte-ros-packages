#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>
#include <tmx_cpp/tmx.hpp>

#include <std_msgs/msg/header.hpp>

class Mirte_module
{
public:
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::string name;

  auto get_header()
  {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }

  Mirte_module(NodeData node_data, std::string name);
};