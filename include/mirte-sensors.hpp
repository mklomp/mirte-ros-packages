#pragma once
#include "mirte-board.hpp"
#include "parsers/sensors.hpp"
#include "ros.hpp"
#include <tmx.hpp>
class Mirte_Sensor;
class Mirte_Sensors {
public:
  Mirte_Sensors(node_handle nh, std::shared_ptr<TMX> tmx,
                std::shared_ptr<Mirte_Board> board,
                std::shared_ptr<Parser> parser);
  std::shared_ptr<TMX> tmx;
  node_handle nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<Mirte_Sensor>> sensors;
};

class Mirte_Sensor {
public:
  std::shared_ptr<TMX> tmx;
  node_handle nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<uint8_t> pins;
  virtual void publish() = 0;
  std::string name;
  auto get_header() {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Sensor(node_handle nh, std::shared_ptr<TMX> tmx,
               std::shared_ptr<Mirte_Board> board, std::vector<uint8_t> pins,
               std::string name);
};

class KeypadMonitor : public Mirte_Sensor {
public:
  KeypadMonitor(node_handle nh, std::shared_ptr<TMX> tmx,
                std::shared_ptr<Mirte_Board> board,
                std::shared_ptr<Keypad_data> keypad_data);
  void publish();
  std::shared_ptr<Keypad_data> keypad_data;
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

  static std::vector<std::shared_ptr<KeypadMonitor>>
  get_keypad_monitors(node_handle nh, std::shared_ptr<TMX> tmx,
                      std::shared_ptr<Mirte_Board> board,
                      std::shared_ptr<Parser> parser);
};

class SonarMonitor : public Mirte_Sensor {
public:
  SonarMonitor(node_handle nh, std::shared_ptr<TMX> tmx,
               std::shared_ptr<Mirte_Board> board,
               std::shared_ptr<Sonar_data> sonar_data);
  void publish();
  std::shared_ptr<Sonar_data> sonar_data;
  publisher<sensor_msgs_range> sonar_pub;
  static std::vector<std::shared_ptr<SonarMonitor>>
  get_sonar_monitors(node_handle nh, std::shared_ptr<TMX> tmx,
                     std::shared_ptr<Mirte_Board> board,
                     std::shared_ptr<Parser> parser);
  void callback(uint16_t value);
  uint16_t value;
  service<mirte_msgs_get_distance> sonar_service;
  bool
  service_callback(const std::shared_ptr<mirte_msgs_get_distance::Request> req,
                   std::shared_ptr<mirte_msgs_get_distance::Response> res);
};

class IntensityMonitor : public Mirte_Sensor {
public:
  IntensityMonitor(node_handle nh, std::shared_ptr<TMX> tmx,
                   std::shared_ptr<Mirte_Board> board, std::vector<pin_t> pins,
                   std::string name)
      : Mirte_Sensor(nh, tmx, board, pins, name) {}
  virtual void publish() = 0;
  static std::vector<std::shared_ptr<IntensityMonitor>>
  get_intensity_monitors(node_handle nh, std::shared_ptr<TMX> tmx,
                         std::shared_ptr<Mirte_Board> board,
                         std::shared_ptr<Parser> parser);
  ~IntensityMonitor() {}
  std::shared_ptr<Intensity_data> intensity_data;
};

class Digital_IntensityMonitor : public IntensityMonitor {
public:
  Digital_IntensityMonitor(node_handle nh, std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::shared_ptr<Intensity_data> intensity_data);
  void publish();
  void callback(uint16_t value);
  bool value;
  publisher<std_msgs_bool> intensity_pub;
};