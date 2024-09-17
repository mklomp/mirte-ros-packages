#include <rclcpp/rclcpp.hpp>

#include <mirte_msgs/srv/set_servo_angle.hpp>

#include <mirte_telemetrix_cpp/actuators/servo.hpp>

#include <tmx_cpp/tmx.hpp>

#include <numbers>
#include <algorithm>
#include <functional>

#include "mirte_telemetrix_cpp/mirte-actuators.hpp"

Servo::Servo(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
             std::shared_ptr<Servo_data> servo_data)
  : Mirte_Actuator(nh, tmx, board, { servo_data->pin }, servo_data->name), data(servo_data)
{
  this->set_angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
      "set_" + name + "_servo_angle",
      std::bind(&Servo::set_angle_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  this->get_range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
      "get_" + name + "_servo_range",
      std::bind(&Servo::get_range_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  tmx->attach_servo(data->pin, data->min_pulse, data->max_pulse);
}

void Servo::set_angle_service_callback(mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
                                       mirte_msgs::srv::SetServoAngle::Response::SharedPtr res)
{
  float angle = req->angle;
  bool is_degrees = req->degrees;

  if (is_degrees == mirte_msgs::srv::SetServoAngle::Request::RADIANS)
  {
    angle = angle * (180.0 / std::numbers::pi);
  }

  if (angle > data->max_angle || angle < data->min_angle)
  {
    RCLCPP_WARN(nh->get_logger(),
                "The provided angle is out of range. Angle %.3f degrees was requested, but range is [%.3f, %.3f]",
                angle, data->min_angle, data->max_angle);
    res->status = false;
    return;
  }

  float fraction = (angle - data->min_angle) / (data->max_angle - data->min_angle);
  uint16_t duty_cycle = (uint16_t)(fraction * (data->max_pulse - data->min_pulse)) + data->min_pulse;

  tmx->write_servo(data->pin, std::clamp(duty_cycle, (uint16_t)data->min_pulse, (uint16_t)data->max_pulse));
  res->status = true;
}

void Servo::get_range_service_callback(const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
                                       mirte_msgs::srv::GetServoRange::Response::SharedPtr res)
{
  res->max = data->max_angle;
  res->min = data->min_angle;
}

Servo::~Servo()
{
  tmx->detach_servo(data->pin);
}

std::vector<std::shared_ptr<Mirte_Actuator>> Servo::get_servos(std::shared_ptr<rclcpp::Node> nh,
                                                                      std::shared_ptr<tmx_cpp::TMX> tmx,
                                                                      std::shared_ptr<Mirte_Board> board,
                                                                      std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<Mirte_Actuator>> servos;
  auto servo_datas = Servo_data::parse_servo_data(parser, board);
  for (auto servo_data : servo_datas)
  {
    if (servo_data->check())
    {
      auto servo = std::make_shared<Servo>(nh, tmx, board, servo_data);
      servos.push_back(servo);
    }
  }
  return servos;
}
