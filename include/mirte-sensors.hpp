#pragma once
#include "mirte-board.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>
#include "mirte_msgs/srv/get_keypad.hpp"
#include "mirte_msgs/msg/keypad.hpp"
#include "std_msgs/msg/header.hpp"
class Mirte_Sensor;
class Mirte_Sensors
{
public:
  Mirte_Sensors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<Mirte_Sensor *> sensors;
  std::vector<uint8_t> resolvePins(rclcpp::Parameter);
};

class Mirte_Sensor
{
public:
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<uint8_t> pins;
  virtual void publish() = 0;
  std::string name;
  auto get_header()
  {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Sensor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
};

class KeypadMonitor : public Mirte_Sensor
{
public:
  KeypadMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::vector<uint8_t> pins, std::string name);
  void publish();
  rclcpp::Publisher<mirte_msgs::msg::Keypad>::SharedPtr keypad_pub;
  rclcpp::Publisher<mirte_msgs::msg::Keypad>::SharedPtr keypad_pressed_pub;
  rclcpp::Service<mirte_msgs::srv::GetKeypad>::SharedPtr keypad_service;
  bool keypad_callback(
    const std::shared_ptr<mirte_msgs::srv::GetKeypad::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetKeypad::Response> res);
  void callback(uint16_t value);
  uint16_t value;
  std::string last_key;
  double last_debounce_time = nh->now().seconds();
  std::string last_debounced_key;

  static std::vector<KeypadMonitor *>
  get_keypad_monitors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board);
};
