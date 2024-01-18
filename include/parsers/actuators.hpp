#pragma once
#include "parsers/parsers.hpp"
#include "mirte-board.hpp"

class Servo_data
{
public:
  std::string name;
  pin_t pin;
  int min_pulse;
  int max_pulse;
  Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse)
  {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
  }
};

class Motor_data
{
public:
  std::string name;
  pin_t P1 = -1;
  pin_t P2 = -1;
  pin_t D1 = -1;
  pin_t D2 = -1;
  bool inverted = false;
  enum class Motor_type
  {
    PP, DP, DDP
  };
  Motor_type type;
  Motor_data(std::string name, std::vector<uint8_t> pins, Motor_type type)
  {
    this->name = name;
    this->type = type;
  }
  Motor_data() {}
};

std::vector<std::shared_ptr<Servo_data>> parse_servo_data(
  std::shared_ptr<Parser> parser,
  std::shared_ptr<Mirte_Board> board);

std::vector<std::shared_ptr<Motor_data>> parse_motor_data(
  std::shared_ptr<Parser> parser,
  std::shared_ptr<Mirte_Board> board);
