#include "mirte-actuators.hpp"

Mirte_Actuators::Mirte_Actuators(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser)
{
  this->tmx = tmx;
  this->nh = nh;
  this->board = board;
  this->actuators = Motor::get_motors(nh, tmx, board);
  parse_servo_data(parser, board);
}

Mirte_Actuator::Mirte_Actuator(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::vector<uint8_t> pins,
  std::string name)
{
  this->tmx = tmx;
  this->nh = nh;
  this->board = board;
  this->pins = pins;
  this->name = name;
}
std::vector<Mirte_Actuator *> Motor::get_motors(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board)
{
  std::vector<Mirte_Actuator *> motors;
  rclcpp::Parameter motors_config;

  // nh.getParam("/mirte/motor", motors_config);
  // for (auto &motor_it : motors_config) {
  //   std::cout << motor_it.first << std::endl;
  //   rclcpp::Parameter motor_config = motor_it.second;
  //   ROS_ASSERT(motor_config.getType() == rclcpp::Parameter::TypeStruct);
  //   std::string type = motor_config["type"];
  //   std::string name = motor_it.first;
  //   std::vector<uint8_t> pins = board.resolvePins(motor_config["pins"]);
  //   if (type == "pp") {
  //     motors.push_back(new PPMotor( nh, tmx, board, pins, name));
  //   } else if (type == "dp") {
  //     motors.push_back(new DPMotor( nh, tmx, board, pins, name));
  //   } else {
  //     ROS_ERROR("Unknown motor type: %s", type.c_str());
  //   }
  // }
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


void Motor::motor_callback(const std_msgs::msg::Int32 & msg)
{
  this->set_speed(msg.data);
}

Motor::Motor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, std::string name)
: Mirte_Actuator(nh, tmx, board, pins, name)
{
  motor_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
    "/mirte/set_" + this->name + "_speed",
    std::bind(&Motor::service_callback, this, std::placeholders::_1, std::placeholders::_2));

  ros_client = nh->create_subscription<std_msgs::msg::Int32>(
    "/mirte/motor_" + this->name + "_speed", 1000,
    std::bind(&Motor::motor_callback, this, std::placeholders::_1) );
}

DPMotor::DPMotor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, std::string name)
: Motor(nh, tmx, board, pins, name)
{
  // ROS_ASSERT(pins.size() == 2);
  tmx->setPinMode(pins[0], TMX::PIN_MODES::DIGITAL_OUTPUT);
  tmx->setPinMode(pins[1], TMX::PIN_MODES::PWM_OUTPUT);
  // motor_service = nh.advertiseService(name, &Motor::service_callback, this);
  // ros_client = nh.subscribe<mirte_msgs::SetMotorSpeed>(name, 1000,
  //                                                      &Motor::motor_callback,
  //                                                      this);
}

void DPMotor::set_speed(int speed)
{
  int32_t speed_ = speed * std::pow(2, 16) / 100;
  tmx->pwmWrite(pins[1], speed > 0 ? speed_ : -speed_);
  std::cout << "1:" << std::dec << speed << std::endl;
  tmx->digitalWrite(pins[0], speed > 0 ? 1 : 0);
  std::cout << "2:" << std::dec << (speed > 0 ? 1 : 0) << std::endl;
  std::cout << "Setting speed to " << std::dec << speed << std::endl;
  // tmx->set_digital_pwm(pins[0], speed);
}

PPMotor::PPMotor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, std::string name)
: Motor(nh, tmx, board, pins, name) {}
void PPMotor::set_speed(int speed)
{
  int32_t speed_ = speed * std::pow(2, 16) / 100;
  tmx->pwmWrite(pins[0], speed > 0 ? speed_ : 0);
  tmx->pwmWrite(pins[1], speed < 0 ? -speed_ : 0);
  std::cout << "1:" << std::dec << (speed < 0 ? -speed_ : 0) << std::endl;
  std::cout << "2:" << std::dec << (speed > 0 ? speed_ : 0) << std::endl;
  std::cout << "Setting speed to " << std::dec << speed << std::endl;

  std::cout << "PP Setting speed to " << std::dec << speed << std::endl;
  // tmx->set_pwm(pins[0], speed);
}
