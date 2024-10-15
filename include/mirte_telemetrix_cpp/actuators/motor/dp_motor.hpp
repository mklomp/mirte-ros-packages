#pragma once

#include <mirte_telemetrix_cpp/actuators/motor.hpp>

class DPMotor : public Motor
{
public:
  DPMotor(NodeData node_data, MotorData motor_data);
  void set_speed(int speed);
  pin_t dir_pin;
  pin_t pwm_pin;
};
