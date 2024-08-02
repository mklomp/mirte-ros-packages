#pragma once
#include "mirte-board.hpp"
#include "parsers/parsers.hpp"

class PCA_Motor_data;
class PCA_Servo_data;
class PCA_data {
public:
  std::string name;
  uint8_t addr;
  uint8_t port;
  pin_t scl;
  pin_t sda;
  int frequency;
  std::vector<std::shared_ptr<PCA_Motor_data>> motors;
  std::vector<std::shared_ptr<PCA_Servo_data>> servos;
  PCA_data(std::string name, uint8_t addr, pin_t scl, pin_t sda, uint8_t port, int frequency,
           std::vector<std::shared_ptr<PCA_Motor_data>> motors,
           std::vector<std::shared_ptr<PCA_Servo_data>> servos) {
    this->name = name;
    this->addr = addr;
    this->port = port;
    this->scl = scl;
    this->sda = sda;
    this->frequency = frequency;
    this->motors = motors;
    this->servos = servos;
  }
  PCA_data() {}
  static std::vector<std::shared_ptr<PCA_data>>
  parse_pca_data(std::shared_ptr<Parser> parser,
                 std::shared_ptr<Mirte_Board> board);
  static std::shared_ptr<PCA_data>
  parse_pca_data_single(std::shared_ptr<Parser> parser,
                        std::shared_ptr<Mirte_Board> board,
                        std::string pca_key);

  bool check() { return true; }
};

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


class Hiwonder_servo_data;
class Hiwonder_bus_data {
public:
  std::string name;
  uint8_t uart_port;
  pin_t tx_pin = (pin_t)-1;
  pin_t rx_pin = (pin_t)-1;
  std::vector<std::shared_ptr<Hiwonder_servo_data>> servos;
  Hiwonder_bus_data(std::string name, uint8_t uart_port, pin_t tx_pin, pin_t rx_pin,
                    std::vector<std::shared_ptr<Hiwonder_servo_data>> servos) {
    this->name = name;
    this->servos = servos;
    this->uart_port = uart_port;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
  }
  Hiwonder_bus_data() {}
  static std::vector<std::shared_ptr<Hiwonder_bus_data>>
  parse_hiwonder_bus_data(std::shared_ptr<Parser> parser,
                          std::shared_ptr<Mirte_Board> board);
  bool check() { 
    // TODO
    return true; }
};

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
  Hiwonder_servo_data(std::string name, uint8_t id, int min_angle_out, int max_angle_out,
                      int home_out, float min_angle_in, float max_angle_in, bool invert,
                      std::string frame_id) {
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



#if 0
class Servo_data
{
public:
  std::string name = "";
  pin_t pin =(pin_t)-1;
  int min_pulse = 0;
  int max_pulse = 0;
  Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse)
  {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
  }
  Servo_data() {}
  static std::vector<std::shared_ptr<Servo_data>> parse_servo_data(
    std::shared_ptr<Parser> parser,
    std::shared_ptr<Mirte_Board> board);
  bool check()
  {
    return pin !=(pin_t)-1 && name != "";
  }
};

class Motor_data
{
public:
  std::string name;
  pin_t P1 =(pin_t)-1;
  pin_t P2 =(pin_t)-1;
  pin_t D1 =(pin_t)-1;
  pin_t D2 =(pin_t)-1;
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

  static std::vector<std::shared_ptr<Motor_data>> parse_motor_data(
    std::shared_ptr<Parser> parser,
    std::shared_ptr<Mirte_Board> board);
  bool check()
  {
    switch (type) {
      case Motor_type::PP:
        return P1 != (pin_t)-1 && P2 != (pin_t)-1 && name != "";
        break;
      case Motor_type::DP:
        return P1 != (pin_t)-1 && D1 !=(pin_t)-1 && name != "";
        break;
      case Motor_type::DDP:
        return P1 !=(pin_t)-1 && D2 !=(pin_t)-1 && D1 !=(pin_t)-1 && name != "";
        break;
      default:
        return false;
        break;
    }
  }
};
#endif