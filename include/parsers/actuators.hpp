#pragma once
#include "mirte-board.hpp"
#include "parsers/parsers.hpp"

class Servo_data {
public:
  std::string name = "";
  pin_t pin = (pin_t)-1;
  int min_pulse = 0;
  int max_pulse = 0;
  // Min/Max angle in degrees
  float min_angle = 0;
  float max_angle = 180;
  Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse, float min_angle, float max_angle) {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
    this->min_angle = min_angle;
    this->max_angle = max_angle;
  }
  Servo_data() {}
  static std::vector<std::shared_ptr<Servo_data>>
  parse_servo_data(std::shared_ptr<Parser> parser,
                   std::shared_ptr<Mirte_Board> board);
  bool check() { return pin != (pin_t)-1 && name != ""; }
};

class Motor_data {
public:
  std::string name;
  pin_t P1 = (pin_t)-1;
  pin_t P2 = (pin_t)-1;
  pin_t D1 = (pin_t)-1;
  pin_t D2 = (pin_t)-1;
  bool inverted = false;
  enum class Motor_type { PP, DP, DDP };
  Motor_type type;
  Motor_data(std::string name, std::vector<uint8_t> pins, Motor_type type) {
    this->name = name;
    this->type = type;
  }
  Motor_data() {}

  static std::vector<std::shared_ptr<Motor_data>>
  parse_motor_data(std::shared_ptr<Parser> parser,
                   std::shared_ptr<Mirte_Board> board);
  bool check() {
    switch (type) {
    case Motor_type::PP:
      return P1 != (pin_t)-1 && P2 != (pin_t)-1 && name != "";
      break;
    case Motor_type::DP:
      return P1 != (pin_t)-1 && D1 != (pin_t)-1 && name != "";
      break;
    case Motor_type::DDP:
      return P1 != (pin_t)-1 && D2 != (pin_t)-1 && D1 != (pin_t)-1 &&
             name != "";
      break;
    default:
      return false;
      break;
    }
  }
};
