#pragma once
#include <atomic>
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <tmx_cpp/modules/HiwonderServo.hpp>
#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder/hiwonder_servo_data.hpp>

#include <mirte_msgs/msg/servo_position.hpp>
#include <std_msgs/msg/header.hpp>
#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Hiwonder_servo {
  public:
    Hiwonder_servo(
      NodeData node_data, std::shared_ptr<HiWonderServoData> servo_data,
      std::shared_ptr<tmx_cpp::HiwonderServo_module> bus, std::string servo_group,
      DeviceData::DeviceDuration duration, rclcpp::CallbackGroup::SharedPtr callback_group);
    std::shared_ptr<HiWonderServoData> servo_data;
    std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod;

    // callbacks from the pico
    void position_cb(tmx_cpp::HiwonderServo_module::Servo_pos & pos);

    // ROS:
    //  set_x_servo_enable
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
    bool enable_service_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // set_x_servo_angle
    rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr angle_service;
    void set_angle_service_callback(
      const std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request> req,
      std::shared_ptr<mirte_msgs::srv::SetServoAngle::Response> res);

    // get_x_servo_range
    rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr range_service;
    bool get_range_service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetServoRange::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetServoRange::Response> res);

    // /servos/x/position publisher
    rclcpp::Publisher<mirte_msgs::msg::ServoPosition>::SharedPtr position_pub;

    // angle calcs
    uint16_t calc_angle_out(float angle_in);
    float calc_angle_in(uint16_t angle_out);

    std_msgs::msg::Header get_header();

  private:
    std::shared_ptr<rclcpp::Node> nh;

    rclcpp::TimerBase::SharedPtr servo_timer;

    std::atomic<float> last_angle = 0.0;
    std::atomic<uint16_t> last_raw = 0;
};