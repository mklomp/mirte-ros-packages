#pragma once
#include "mirte-actuators.hpp"
#include "mirte-board.hpp"
#include "mirte-ping.hpp"
#include "mirte-sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <tmx.hpp>

class mirte_node : public rclcpp::Node {
private:
  /* data */

public:
  mirte_node(/* args */);
  ~mirte_node();
  void start(std::shared_ptr<rclcpp::Node> s_node);
  std::shared_ptr<Mirte_Board> s_board;
  std::shared_ptr<TMX> s_tmx;
  std::shared_ptr<Parser> p_s;
  std::shared_ptr<Mirte_Sensors> monitor;
  std::shared_ptr<Mirte_Actuators> actuators;
  std::shared_ptr<Mirte_Ping> ping;
  std::shared_ptr<Mirte_modules> modules;
};
