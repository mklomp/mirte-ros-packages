#include <mirte_telemetrix_cpp/mirte-actuators.hpp>
#include <tmx_cpp/tmx.hpp>

Mirte_Actuators::Mirte_Actuators(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
: nh(nh), tmx(tmx), board(board)
{
  this->set_pin_value_service = nh->create_service<mirte_msgs::srv::SetPinValue>(
    "set_pin_value", std::bind(
                       &Mirte_Actuators::set_pin_value_service_callback, this,
                       std::placeholders::_1, std::placeholders::_2));

  this->actuators = Motor::get_motors(nh, tmx, board, parser);
  // WIP:
  // TODO: set_pin!!!
  // auto servos = Servo_data::parse_servo_data(parser, board);
  // auto motors = Motor_data::parse_motor_data(parser, board);
  auto servos = Servo::get_servos(nh, tmx, board, parser);
  this->actuators.insert(this->actuators.end(), servos.begin(), servos.end());
}

void Mirte_Actuators::set_pin_value_service_callback(
  const mirte_msgs::srv::SetPinValue::Request::ConstSharedPtr req,
  mirte_msgs::srv::SetPinValue::Response::SharedPtr res)
{
  bool is_digital = starts_with(req->type, "d") || starts_with(req->type, "D");
  auto pin = this->board->resolvePin(req->pin);

  if (is_digital) {
    this->tmx->setPinMode(pin, TMX::PIN_MODES::DIGITAL_OUTPUT, false, 0);
    // TODO: Maybe a sleep is required here?
    this->tmx->digitalWrite(pin, req->value != 0);
  } else {
    this->tmx->setPinMode(pin, TMX::PIN_MODES::PWM_OUTPUT, false, 0);
    // TODO: Maybe a sleep is required here?
    this->tmx->pwmWrite(pin, std::clamp(req->value, 0, this->board->get_max_pwm()));
  }
  res->status = true;
}

Mirte_Actuator::Mirte_Actuator(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::vector<pin_t> pins, std::string name)
: nh(nh), tmx(tmx), board(board), pins(pins), name(name)
{
}
