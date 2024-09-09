#pragma once

#define TMX_ROS_VERSION 2
#if TMX_ROS_VERSION == 2
#include <rclcpp/rclcpp.hpp>

#include "mirte_msgs/msg/keypad.hpp"
#include "mirte_msgs/srv/get_keypad.hpp"
#include "std_msgs/msg/header.hpp"
// using std::shared_ptr<rclcpp::Node> = std::shared_ptr<rclcpp::Node>;
// template <typename T> using publisher =
// std::shared_ptr<rclcpp::Publisher<T>>;

// template <typename T>
// using subscriber = std::shared_ptr<rclcpp::Subscription<T>>;

// template <typename T> using service = std::shared_ptr<rclcpp::Service<T>>;

//  message types
#include "sensor_msgs/msg/range.hpp"
// #include "sensor_msgs/msg/"
// using sensor_msgs::msg::Range = sensor_msgs::msg::Range;

#include "mirte_msgs/srv/get_distance.hpp"
// using mirte_msgs::srv::GetDistance = mirte_msgs::srv::GetDistance;

// #include "std_msgs/msg/bool.hpp"
// using std_msgs_bool = std_msgs::msg::Bool;

#include "mirte_msgs/msg/intensity_digital.hpp"
// using mirte_msgs::msg::IntensityDigital = mirte_msgs::msg::IntensityDigital;

#include "mirte_msgs/msg/intensity.hpp"
// using mirte_msgs::msg::Intensity = mirte_msgs::msg::Intensity;

#include "mirte_msgs/srv/get_intensity_digital.hpp"
// using mirte_msgs::srv::GetIntensityDigital =
// mirte_msgs::srv::GetIntensityDigital;

#include "mirte_msgs/srv/get_intensity.hpp"
// using mirte_msgs::srv::GetIntensity = mirte_msgs::srv::GetIntensity;

#include "mirte_msgs/srv/get_pin_value.hpp"
// using mirte_msgs::srv::GetPinValue = mirte_msgs::srv::GetPinValue;

#elif TMX_ROS_VERSION == 1

#endif
