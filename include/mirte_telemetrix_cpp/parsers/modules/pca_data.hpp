#pragma once

#include <string>

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>
#include <mirte_telemetrix_cpp/mirte-board.hpp>

class PCA_Motor_data;
class PCA_Servo_data;

class PCAData : public I2CModuleData {
public:
  // uint8_t addr = 0x41;
  // uint8_t port = 0xFF;

  // pin_t scl = 0xFF;
  // pin_t sda = 0xFF;
  int frequency = 2000;

  std::vector<std::shared_ptr<PCA_Motor_data>> motors;
  std::vector<std::shared_ptr<PCA_Servo_data>> servos;

  PCAData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters);
  
  bool check();

  static std::string get_module_type() { return "pca9685";}
};

// class PCA_data {
// public:
//   std::string name;
//   uint8_t addr = 0x41;
//   uint8_t port = 0xFF;
//   pin_t scl = 0xFF;
//   pin_t sda = 0xFF;
//   int frequency = 2000;
//   std::vector<std::shared_ptr<PCA_Motor_data>> motors;
//   std::vector<std::shared_ptr<PCA_Servo_data>> servos;
//   PCA_data(std::string name, uint8_t addr, pin_t scl, pin_t sda, uint8_t port,
//            int frequency, std::vector<std::shared_ptr<PCA_Motor_data>> motors,
//            std::vector<std::shared_ptr<PCA_Servo_data>> servos) {
//     this->name = name;
//     this->addr = addr;
//     this->port = port;
//     this->scl = scl;
//     this->sda = sda;
//     this->frequency = frequency;
//     this->motors = motors;
//     this->servos = servos;
//   }
//   PCA_data() {}
//   static std::vector<std::shared_ptr<PCA_data>>
//   parse_pca_data(std::shared_ptr<Parser> parser,
//                  std::shared_ptr<Mirte_Board> board);
//   static std::shared_ptr<PCA_data>
//   parse_pca_data_single(std::shared_ptr<Parser> parser,
//                         std::shared_ptr<Mirte_Board> board,
//                         std::string pca_key);

//   bool check() { return true; }
// };

class PCA_Motor_data {
public:
  std::string name;
  pin_t pinA = (pin_t)-1;
  pin_t pinB = (pin_t)-1;
  bool invert = false;
  PCA_Motor_data(std::string name, pin_t pinA, pin_t pinB, bool invert) {
    this->name = name;
    this->pinA = pinA;
    this->pinB = pinB;
    this->invert = invert;
  }
  PCA_Motor_data() {}
  static std::vector<std::shared_ptr<PCA_Motor_data>>
  parse_pca_motor_data(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board, std::string pca_key);
  bool check() { return pinA != (pin_t)-1 && pinB != (pin_t)-1 && name != ""; }
};

class PCA_Servo_data {
public:
  std::string name;
  pin_t pin = (pin_t)-1;
  int min_pulse = 0;
  int max_pulse = 0;
  PCA_Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse) {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
  }
  PCA_Servo_data() {}
  static std::vector<std::shared_ptr<PCA_Servo_data>>
  parse_pca_servo_data(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board,
                       std::string pca_name);
  bool check() { return pin != (pin_t)-1 && name != ""; }
};
