#pragma once
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-actuators.hpp>
#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/mirte-modules.hpp>
#include <mirte_telemetrix_cpp/mirte-sensors.hpp>

class TelemetrixNode : public rclcpp::Node
 {
private:
  /* data */

public:
  TelemetrixNode(const rclcpp::NodeOptions & options);

  ~TelemetrixNode();
  bool start();
  std::shared_ptr<Mirte_Board> board;
  std::shared_ptr<tmx_cpp::TMX> tmx;

  std::shared_ptr<Mirte_Sensors> monitor;
  std::shared_ptr<Mirte_Actuators> actuators;
  std::shared_ptr<Mirte_modules> modules;
};
