#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>

#include <memory>
#include <string>
#include "mirte_telemetrix_cpp/mirte-actuators.hpp"
#include "mirte_telemetrix_cpp/parsers/actuators.hpp"

class Servo : public Mirte_Actuator
{
public:
  Servo(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Servo_data> servo_data);

  // TODO: Maybe make private
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr set_angle_service;
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr get_range_service;

  static std::vector<std::shared_ptr<Mirte_Actuator>> get_servos(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Parser> parser);

  ~Servo();

private:
  void set_angle_service_callback(
    const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngle::Response::SharedPtr res);

  void get_range_service_callback(
    const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetServoRange::Response::SharedPtr res);

  std::shared_ptr<Servo_data> data;

  // TODO: Maybe add deregister just like Python version
};