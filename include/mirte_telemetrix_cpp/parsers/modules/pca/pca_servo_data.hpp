#pragma once
#include <memory>
#include <set>
#include <string>

// #include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class PCA_Servo_data
{
public:
  std::string name;
  pin_t pin = (pin_t)-1;
  int min_pulse = 0;
  int max_pulse = 0;
  PCA_Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse)
  {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
  }
  PCA_Servo_data() {}
  static std::vector<std::shared_ptr<PCA_Servo_data>> parse_pca_servo_data(
    std::shared_ptr<Parser> parser, /*std::shared_ptr<Mirte_Board> board,*/ std::string pca_name,
    std::set<std::string> & unused_keys);
  bool check() { return pin != (pin_t)-1 && name != ""; }
};
