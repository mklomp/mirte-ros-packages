#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/actuators/servo/servo.hpp>
#include <mirte_telemetrix_cpp/mirte-actuators.hpp>

Mirte_Actuators::Mirte_Actuators(NodeData node_data, std::shared_ptr<Parser> parser)
: tmx(node_data.tmx), nh(node_data.nh), board(node_data.board)
{
  this->set_pin_value_service = nh->create_service<mirte_msgs::srv::SetPinValue>(
    "set_pin_value", std::bind(
                       &Mirte_Actuators::set_pin_value_service_callback, this,
                       std::placeholders::_1, std::placeholders::_2));

  this->actuators = Motor::get_motors(node_data, parser);

  auto servos = Servo::get_servos(node_data, parser);
  this->actuators.insert(this->actuators.end(), servos.begin(), servos.end());
}

void Mirte_Actuators::set_pin_value_service_callback(
  const mirte_msgs::srv::SetPinValue::Request::ConstSharedPtr req,
  mirte_msgs::srv::SetPinValue::Response::SharedPtr res)
{
  bool is_digital = starts_with(req->type, "d") || starts_with(req->type, "D");
  auto pin = board->resolvePin(req->pin);

  if (is_digital) {
    tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT, false, 0);
    // TODO: Maybe a sleep is required here?
    tmx->digitalWrite(pin, req->value != 0);
  } else {
    tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT, false, 0);
    // TODO: Maybe a sleep is required here?
    tmx->pwmWrite(pin, std::clamp(req->value, 0, board->get_max_pwm()));
  }
  res->status = true;
}
