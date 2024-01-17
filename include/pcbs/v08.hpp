#include <map>
#include <vector>
#include <string>

constexpr std::map<std::string, std::string> generate_motor_mapping(std::string pin1, std::string pin2)
{
  return {{"pin1", pin1}, {"pin2", pin2}};
}

std::map<std::string, std::map<std::string, std::string>> mirte_pico_pcb_map08 = {
  "IR1" : {"digital" : "16", "analog" : "26"},
  "IR2" : {"digital" : "17", "analog" : "27"},
  "SRF1" : {"trigger" : "7", "echo" : "6"},
  "SRF2" : {"trigger" : "9", "echo" : "8"},
  "I2C1" : {"scl" : "5", "sda" : "4"},
  "I2C2" : {"scl" : "11", "sda" : "10"},
  "ENC1" : {"pin" : "15"},
  "ENC2" : {"pin" : "14"},
  "Keypad" : {"pin" : "28"},
  "Servo1" : {"pin" : "14"},
  "Servo2" : {"pin" : "15"},
  "Servo3" : {"pin" : "12"},
  "Servo4" : {"pin" : "13"},
  "LED" : {"pin" : "25"},  // Does not work with the Pico W
  "MC1-A" : generate_motor_mapping("19", "18"),
  "MC1-B" : generate_motor_mapping("21", "20"),
  "MC2-A" : generate_motor_mapping("16", "26"),
  "MC2-B" : generate_motor_mapping("17", "27"),
}
