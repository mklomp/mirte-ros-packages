#include <map>
#include <string>
using pin_map = std::map<std::string, std::string>;
using connector_map = std::map<std::string, pin_map>;

#ifndef MIRTE_BOARD_HPP
#define MIRTE_BOARD_HPP

#include "mirte_telemetrix_cpp/parsers/parsers.hpp"
#include "mirte_telemetrix_cpp/pcbs/v06.hpp"
#include "mirte_telemetrix_cpp/pcbs/v08.hpp"
#include <map>    // for map, map<>::value_compare
#include <memory> // for shared_ptr, __shared_ptr_access
#include <string> // for string, basic_string, operator<

class Mirte_Board {
public: 
  virtual int get_adc_bits() = 0;
  virtual int get_max_pwm() = 0;
  virtual int resolvePin(std::string pin) = 0;
  virtual std::map<std::string, int>
  resolveConnector(std::string connector) = 0;
  virtual uint8_t resolveI2CPort(uint8_t sda) = 0;
  virtual uint8_t resolveUARTPort(uint8_t pin) = 0;
  static std::shared_ptr<Mirte_Board> create(std::shared_ptr<Parser> parser);
  virtual ~Mirte_Board() {}
};

class Mirte_Board_atmega328p : public Mirte_Board {
public:
  Mirte_Board_atmega328p();
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits();
  int get_max_pwm() { return 255; }
  virtual uint8_t resolveI2CPort(uint8_t sda) override { return 0; }

  virtual uint8_t resolveUARTPort(uint8_t pin) override { return 0; };
};
class Mirte_Board_pico : public Mirte_Board {
public:
  // Mirte_Board_pico(
  //   // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh
  //   );
  Mirte_Board_pico();
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits() { 
    /*   # NOTE: the pico itself has 12 bits, but micropython will upgrade to 16
    # no clue why 14 works*/
    return 14; }
  int get_max_pwm() { return 20000; } // TODO: check with actual board
  virtual uint8_t resolveI2CPort(uint8_t sda) override;
  virtual uint8_t resolveUARTPort(uint8_t pin) override;
};

class Mirte_Board_pcb : public Mirte_Board {
public:
  Mirte_Board_pcb(std::shared_ptr<Mirte_Board> mcu, std::string version);
  // Mirte_Board_pcb(
  //   // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh,
  //   std::shared_ptr<Mirte_Board> mcu);
  std::shared_ptr<Mirte_Board> mcu; // to look up pins
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits() { return mcu->get_adc_bits(); }
  int get_max_pwm() { return mcu->get_max_pwm(); }
  virtual uint8_t resolveI2CPort(uint8_t sda) override;
  virtual uint8_t resolveUARTPort(uint8_t pin) override;
  std::string version;
  connector_map connectors = mirte_pico_pcb_map08;
};

#endif