#pragma once
#include "mirte-board.hpp"
#include "parsers/sensors.hpp"
#include "ros.hpp"
#include <tmx.hpp>
class Mirte_Sensor;
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

class Mirte_Sensor {
public:
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<uint8_t> pins;
  virtual void publish() = 0;
  std::string name;
  virtual void stop() {}
  auto get_header() {
    std_msgs::msg::Header header;
    header.stamp = nh->now();

    return header;
  }
  Mirte_Sensor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
               std::shared_ptr<Mirte_Board> board, std::vector<uint8_t> pins,
               std::string name);
};

class KeypadMonitor : public Mirte_Sensor {
public:
  KeypadMonitor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                std::shared_ptr<Mirte_Board> board,
                std::shared_ptr<Keypad_data> keypad_data);
  void publish();
  std::shared_ptr<Keypad_data> keypad_data;
  std::shared_ptr<rclcpp::Publisher<mirte_msgs::msg::Keypad>> keypad_pub;
  std::shared_ptr<rclcpp::Publisher<mirte_msgs::msg::Keypad>>
      keypad_pressed_pub;
  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetKeypad>> keypad_service;
  bool keypad_callback(
      const std::shared_ptr<mirte_msgs::srv::GetKeypad::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetKeypad::Response> res);
  void callback(uint16_t value);
  uint16_t value;
  std::string last_key;
  double last_debounce_time = nh->now().seconds();
  std::string last_debounced_key;

  static std::vector<std::shared_ptr<KeypadMonitor>> get_keypad_monitors(
      std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
      std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser);
};

class SonarMonitor : public Mirte_Sensor {
public:
  SonarMonitor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
               std::shared_ptr<Mirte_Board> board,
               std::shared_ptr<Sonar_data> sonar_data);
  void publish();
  std::shared_ptr<Sonar_data> sonar_data;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> sonar_pub;
  static std::vector<std::shared_ptr<SonarMonitor>>
  get_sonar_monitors(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                     std::shared_ptr<Mirte_Board> board,
                     std::shared_ptr<Parser> parser);
  void callback(uint16_t value);
  uint16_t value;
  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetDistance>> sonar_service;
  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetDistance::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetDistance::Response> res);
};

class IntensityMonitor : public Mirte_Sensor {
public:
  IntensityMonitor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                   std::shared_ptr<Mirte_Board> board, std::vector<pin_t> pins,
                   std::string name)
      : Mirte_Sensor(nh, tmx, board, pins, name) {}
  virtual void publish() = 0;
  static std::vector<std::shared_ptr<IntensityMonitor>> get_intensity_monitors(
      std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
      std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser);
  ~IntensityMonitor() {}
  std::shared_ptr<Intensity_data> intensity_data;
};

class Digital_IntensityMonitor : public IntensityMonitor {
public:
  Digital_IntensityMonitor(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::shared_ptr<Intensity_data> intensity_data);
  void publish();
  void callback(uint16_t value);
  bool value;
  std::shared_ptr<rclcpp::Publisher<mirte_msgs::msg::IntensityDigital>>
      intensity_pub;

  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetIntensityDigital>>
      intensity_service;
  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res);
};

class Analog_IntensityMonitor : public IntensityMonitor {
public:
  Analog_IntensityMonitor(std::shared_ptr<rclcpp::Node> nh,
                          std::shared_ptr<TMX> tmx,
                          std::shared_ptr<Mirte_Board> board,
                          std::shared_ptr<Intensity_data> intensity_data);
  void publish();
  void callback(uint16_t value);
  uint16_t value;
  std::shared_ptr<rclcpp::Publisher<mirte_msgs::msg::Intensity>> intensity_pub;
  std::shared_ptr<rclcpp::Service<mirte_msgs::srv::GetIntensity>>
      intensity_service;
  bool service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res);
};