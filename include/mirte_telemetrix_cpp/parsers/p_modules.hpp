#pragma once
#include "mirte_telemetrix_cpp/mirte-board.hpp"
#include "mirte_telemetrix_cpp/parsers/parsers.hpp"

class Hiwonder_servo_data;
// class Hiwonder_bus_data {
// public:
//   std::string name;
//   uint8_t uart_port;
//   pin_t tx_pin = (pin_t)-1;
//   pin_t rx_pin = (pin_t)-1;
//   std::vector<std::shared_ptr<Hiwonder_servo_data>> servos;
//   Hiwonder_bus_data(std::string name, uint8_t uart_port, pin_t tx_pin,
//                     pin_t rx_pin,
//                     std::vector<std::shared_ptr<Hiwonder_servo_data>> servos) {
//     this->name = name;
//     this->servos = servos;
//     this->uart_port = uart_port;
//     this->tx_pin = tx_pin;
//     this->rx_pin = rx_pin;
//   }
//   Hiwonder_bus_data() {}
//   static std::vector<std::shared_ptr<Hiwonder_bus_data>>
//   parse_hiwonder_bus_data(std::shared_ptr<Parser> parser,
//                           std::shared_ptr<Mirte_Board> board);
//   static std::shared_ptr<Hiwonder_bus_data>
//   parse_single(std::shared_ptr<Parser> parser,
//                std::shared_ptr<Mirte_Board> board, std::string bus_key);
//   bool check() {
//     // TODO
//     return true;
//   }
// };

class Hiwonder_servo_data {
public:
  std::string name;
  uint8_t id;
  int min_angle_out = 0;
  int max_angle_out = 24000;
  int home_out = 1000;
  float min_angle_in = -1;
  float max_angle_in = -1;
  bool invert = false;
  std::string frame_id;
  Hiwonder_servo_data(std::string name, uint8_t id, int min_angle_out,
                      int max_angle_out, int home_out, float min_angle_in,
                      float max_angle_in, bool invert, std::string frame_id) {
    this->name = name;
    this->id = id;
    this->min_angle_out = min_angle_out;
    this->max_angle_out = max_angle_out;
    this->home_out = home_out;
    this->min_angle_in = min_angle_in;
    this->max_angle_in = max_angle_in;
    this->invert = invert;
    this->frame_id = frame_id;
  }
  Hiwonder_servo_data() {}
  static std::vector<std::shared_ptr<Hiwonder_servo_data>>
  parse_hiwonder_servo_data(std::shared_ptr<Parser> parser,
                            std::shared_ptr<Mirte_Board> board,
                            std::string bus_name);
  bool check() { return name != ""; }
};

// class INA226_data {
// public:
//   std::string name;
//   uint8_t addr = 64;
//   uint8_t port = 0;
//   float max_current = 10;
//   float max_voltage = 14;
//   float min_voltage = 10.5;
//   float power_low_time = 5;
//   pin_t scl = 0xFF;
//   pin_t sda = 0xFF;
//   INA226_data(std::string name, uint8_t addr, uint8_t port, float max_current,
//               float min_voltage, float max_voltage) {
//     this->name = name;
//     this->addr = addr;
//     this->port = port;
//     this->max_current = max_current;
//     this->max_voltage = max_voltage;
//   }
//   INA226_data() {}
//   static std::vector<std::shared_ptr<INA226_data>>
//   parse_ina226_data(std::shared_ptr<Parser> parser,
//                     std::shared_ptr<Mirte_Board> board);
//   static std::shared_ptr<INA226_data>
//   parse_ina226_data_single(std::shared_ptr<Parser> parser,
//                            std::shared_ptr<Mirte_Board> board,
//                            std::string ina226_key);
//   bool check() { return name != "" && addr != 0; }
// };