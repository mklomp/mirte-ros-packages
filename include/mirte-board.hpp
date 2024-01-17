#pragma once
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>
#include <memory>
#include <vector>
#include "util.hpp"

class Mirte_Board
{
public:
  Mirte_Board(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh);
  std::shared_ptr<TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  virtual int get_adc_bits() = 0;
  // std::vector<uint8_t> resolvePins(std::string pin);
  virtual int resolvePin(std::string pin) = 0;
  virtual std::map<std::string, int> resolveConnector(std::string connector) = 0;
};

class Mirte_Board_atmega328p : public Mirte_Board
{
public:
  Mirte_Board_atmega328p(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh);
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits()
  {
    return 12;
  }
};

class Mirte_Board_pico : public Mirte_Board
{
public:
  Mirte_Board_pico(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh);
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits() {return 12;}
};

class Mirte_Board_pcb : public Mirte_Board
{
public:
  Mirte_Board_pcb(std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<Mirte_Board> mcu);
  std::shared_ptr<Mirte_Board> mcu; // to look up pins
  std::map<std::string, int> resolveConnector(std::string connector);
  int resolvePin(std::string pin);
  int get_adc_bits() {return mcu->get_adc_bits();}

};
