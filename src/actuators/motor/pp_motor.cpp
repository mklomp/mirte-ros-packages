#include <mirte_telemetrix_cpp/actuators/motor/pp_motor.hpp>

PPMotor::PPMotor(NodeData node_data, MotorData motor_data)
: Motor(node_data, {motor_data.P1, motor_data.P2}, motor_data)
{
  this->pwmA_pin = motor_data.P1;
  this->pwmB_pin = motor_data.P2;
  std::cout << "PPMotor" << std::hex << this->pwmA_pin << " " << std::hex << this->pwmB_pin
            << std::endl;
  tmx->setPinMode(this->pwmA_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
  tmx->setPinMode(this->pwmB_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
}

void PPMotor::setA(int speed) { tmx->pwmWrite(this->pwmA_pin, speed); }
void PPMotor::setB(int speed) { tmx->pwmWrite(this->pwmB_pin, speed); }

void PPMotor::set_speed(int speed)
{
  int32_t speed_ = (int32_t)((float)speed * (this->max_pwm) / 100.0);

  this->setA(speed > 0 ? speed_ : 0);
  this->setB(speed < 0 ? -speed_ : 0);
  std::cout << "1:" << std::dec << (speed < 0 ? -speed_ : 0) << std::endl;
  std::cout << "2:" << std::dec << (speed > 0 ? speed_ : 0) << std::endl;
  std::cout << "Setting speed to " << std::dec << speed << std::endl;

  std::cout << "PP Setting speed to " << std::dec << speed << std::endl;
  // tmx->set_pwm(pins[0], speed);
}
