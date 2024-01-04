#pragma once
#include <ros/ros.h>
#include <tmx.hpp>
#include <vector>
#include <thread>
#include <functional>
class Mirte_Ping {
public:
  Mirte_Ping(TMX &tmx, ros::NodeHandle &nh, std::function<void()> stop_func);
  TMX *tmx;
  ros::NodeHandle *nh;
  void ping_task();
  std::thread ping_thread; // TODO: jthread from c++20
  void ping_callback(std::vector<uint8_t> pins);
  uint8_t last_ping = 0;
  uint8_t magic = 0;
  bool first_magic = true;
  std::function<void()> stop_func;
  void stop();
  bool is_stopped = false;
};