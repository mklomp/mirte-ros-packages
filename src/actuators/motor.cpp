#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/mirte-actuators.hpp>

std::vector<std::shared_ptr<Mirte_Actuator>> Motor::get_motors(
  NodeData node_data, std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<Mirte_Actuator>> motors;
  auto motor_datas = parse_all<MotorData>(parser, node_data.board);
  for (auto motor_data : motor_datas) {
    if (motor_data.check()) {
      if (motor_data.type == MotorData::MotorType::PP) {
        auto motor = std::make_shared<PPMotor>(node_data, motor_data);
        motor->start();
        motors.push_back(motor);
      } else if (motor_data.type == MotorData::MotorType::DP) {
        auto motor = std::make_shared<DPMotor>(node_data, motor_data);
        motor->start();
        motors.push_back(motor);
      } else if (motor_data.type == MotorData::MotorType::DDP) {
        // motors.push_back(std::make_shared<DDPMotor>(
        //     nh, tmx, board, motor_data,
        //     motor_data->name));
        RCLCPP_WARN(node_data.nh->get_logger(), "TODO: DDP MOTOR");
      }
    }
  }
  return motors;
}
bool Motor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res)
{
  this->set_speed(req->speed);
  res->status = true;
  return true;
}

void Motor::motor_callback(const std_msgs::msg::Int32 & msg) { this->set_speed(msg.data); }

Motor::Motor(NodeData node_data, std::vector<pin_t> pins, MotorData motor_data)
: Mirte_Actuator(node_data, pins, (DeviceData)motor_data), data(motor_data)
{
  motor_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
    "set_" + this->name + "_speed",
    std::bind(&Motor::service_callback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  rclcpp::SubscriptionOptions options;
  options.callback_group = this->callback_group;
  ros_client = nh->create_subscription<std_msgs::msg::Int32>(
    "motor_" + this->name + "_speed", rclcpp::SystemDefaultsQoS(),
    std::bind(&Motor::motor_callback, this, std::placeholders::_1), options);

  this->max_pwm = board->get_max_pwm();
}

DPMotor::DPMotor(NodeData node_data, MotorData motor_data)
: Motor(node_data, {motor_data.D1, motor_data.P1}, motor_data)
{
  this->pwm_pin = motor_data.P1;
  this->dir_pin = motor_data.D1;
  tmx->setPinMode(this->pwm_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
  tmx->setPinMode(this->dir_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT);
  // motor_service = nh.advertiseService(name, &Motor::service_callback, this);
  // ros_client = nh.subscribe<mirte_msgs::SetMotorSpeed>(name, 1000,
  //                                                      &Motor::motor_callback,
  //                                                      this);
}

void DPMotor::set_speed(int speed)
{
  int32_t speed_ = (int32_t)((float)speed * (this->max_pwm) / 100.0);
  tmx->pwmWrite(this->pwm_pin, speed > 0 ? speed_ : -speed_);
  std::cout << "1:" << std::dec << speed << std::endl;
  tmx->digitalWrite(this->dir_pin, speed > 0 ? 1 : 0);
  std::cout << "2:" << std::dec << (speed > 0 ? 1 : 0) << std::endl;
  std::cout << "Setting speed to " << std::dec << speed << std::endl;
  // tmx->set_digital_pwm(pins[0], speed);
}

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
