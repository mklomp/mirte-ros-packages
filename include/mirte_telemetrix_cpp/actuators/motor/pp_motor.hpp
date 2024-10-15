#pragma once

#include <mirte_telemetrix_cpp/actuators/motor.hpp>

class PPMotor : public Motor
{
public:
  PPMotor(NodeData node_data, MotorData motor_data);

  // PPMotor();  // Only for PCA_motor
  void set_speed(int speed);
  pin_t pwmA_pin;
  pin_t pwmB_pin;
  void setA(int speed);
  void setB(int speed);
};