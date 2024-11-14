#include <map>
#include <string>
#include <vector>

#include "mirte_telemetrix_cpp/mirte-board.hpp"

// TODO: MAYBE ADD UNDEFINE
#define generate_motor_mapping(pin1, pin2)                                     \
  {                                                                            \
    {"P1", pin1}, {"P2", pin2}, {"D1", pin2}, { "D2", pin1 }                   \
  }

// TODO: ADD OD1 and OD2
const connector_map mirte_pico_pcb_map06 = {
    {"IR1", {{"digital", "16"}, {"analog", "26"}}},
    {"IR2", {{"digital", "17"}, {"analog", "27"}}},
    {"SRF1", {{"trigger", "7"}, {"echo", "6"}}},
    {"SRF2", {{"trigger", "9"}, {"echo", "8"}}},
    {"I2C1", {{"scl", "5"}, {"sda", "4"}}},
    {"I2C2", {{"scl", "11"}, {"sda", "10"}}},
    {"ENC1", {{"pinA", "15"}, {"pinB", "-1"}}},
    {"ENC2", {{"pinA", "14"}, {"pinB", "-1"}}},
    {"Keypad", {{"pin", "28"}}},
    {"Servo1", {{"pin", "2"}}}, // These 2 servos don't work together with the
                                // motor controllers at the same time
    {"Servo2", {{"pin", "3"}}}, // # These 2 servos don't work together with the
                                // motor controllers at the same time
    {"Servo3", {{"pin", "12"}}},
    {"Servo4", {{"pin", "13"}}},
    // FIXME: ONBOARD LED OR BOARD LED?
    {"LED", {{"pin", "25"}}},
    {"MC1-A", generate_motor_mapping("19", "18")},
    {"MC1-B", generate_motor_mapping("21", "20")},
    {"MC2-A", generate_motor_mapping("17", "27")},
    {"MC2-B", generate_motor_mapping("16", "26")}};

// const connector_map mirte_pico_pcb_map06 = {
//   // TODO
//   {"IR1", {{"digital", "16"}, {"analog", "26"}}},
//   {"IR2", {{"digital", "17"}, {"analog", "27"}}},
//   {"SRF1", {{"trigger", "7"}, {"echo", "6"}}},
//   {"SRF2", {{"trigger", "9"}, {"echo", "8"}}},
//   {"I2C1", {{"scl", "5"}, {"sda", "4"}}},
//   {"I2C2", {{"scl", "11"}, {"sda", "10"}}},
//   {"ENC1", {{"pin", "15"}}},
//   {"ENC2", {{"pin", "14"}}},
//   {"Keypad", {{"pin", "28"}}},
//   {"Servo1", {{"pin", "14"}}},
//   {"Servo2", {{"pin", "15"}}},
//   {"Servo3", {{"pin", "12"}}},
//   {"Servo4", {{"pin", "13"}}},
//   {"LED", {{"pin", "25"}}}, // Does not work with the Pico W
//   {"MC1-A", generate_motor_mapping("19", "18")},
//   {"MC1-B", generate_motor_mapping("21", "20")},
//   {"MC2-A", generate_motor_mapping("16", "26")},
//   {"MC2-B", generate_motor_mapping("17", "7")},
// };
