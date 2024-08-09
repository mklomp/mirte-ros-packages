// https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
// Roughly based on:
// https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/include/my_robot_hw_interface.h
// https://github.com/PickNikRobotics/ros_control_boilerplate
// https://github.com/DeborggraeveR/ampru

// https://github.com/resibots/dynamixel_control_hw/blob/master/include/dynamixel_control_hw/hardware_interface.hpp
// https://github.com/FRC900/2018RobotCode/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frcrobot_hw_interface.h
// INTERESTING CLEAN ONE:
// https://github.com/ros-controls/ros_controllers/blob/indigo-devel/diff_drive_controller/test/diffbot.h

#pragma once
#define _USE_MATH_DEFINES

// ROS
#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
// ros_control
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ostringstream
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>

#include <boost/format.hpp>
#include <chrono>
#include <control_toolbox/pid.hpp>
#include <future>
#include <mutex>
#include <thread>
// const unsigned int NUM_JOINTS = 4;
const auto service_format = "mirte/set_%s_speed";
const auto encoder_format = "mirte/encoder/%s";
const auto max_speed = 100; // Quick fix hopefully for power dip.

namespace mirte_base_control {

class MirteBaseHWInterface : public hardware_interface::SystemInterface {
public:
  MirteBaseHWInterface();

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  // on_configure
  // on_cleanup
  // on_shutdown
  // on_activate
  // on_deactivate
  // on_error
  // on_init
  // export_state_interfaces
  // export_command_interfaces
  // read
  // write
  double calc_speed_pid(int joint, double target,
                        const rclcpp::Duration &period);

  double calc_speed_map(int joint, double target,
                        const rclcpp::Duration &period);

  bool write_single(int joint, double speed, const rclcpp::Duration &period);
  /*
   *
   */
  void write(const rclcpp::Duration &period);

  double rad_per_enc_tick() { return 2.0 * M_PI / this->ticks; }
  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read_single(int joint, const rclcpp::Duration &period);
  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read(const rclcpp::Duration &period);

  rclcpp::Node nh;
  rclcpp::Node private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  bool running_ = true;
  double _wheel_diameter;
  double _max_speed;
  double ticks = 40.0;

  std::vector<int> _wheel_encoder;
  std::vector<rclcpp::Time> _wheel_encoder_update_time;
  std::vector<double> _last_cmd;
  std::vector<double> _last_sent_cmd;
  std::vector<int> _last_value;
  std::vector<int> _last_wheel_cmd_direction;

  rclcpp::Time curr_update_time, prev_update_time;

  std::vector<rclcpp::Subscriber> wheel_encoder_subs_;
  rclcpp::ServiceServer start_srv_;
  rclcpp::ServiceServer stop_srv_;

  std::vector<rclcpp::ServiceClient> service_clients;
  std::vector<mirte_msgs::SetMotorSpeed> service_requests;
  std::vector<std::string> joints;
  bool enablePID;
  std::vector<std::shared_ptr<control_toolbox::Pid>> pids;
  std::shared_ptr<control_toolbox::Pid>
      reconfig_pid; // one dummy pid to use for the dynamic reconfigure

  bool start_callback(std_srvs::Empty::Request & /*req*/,
                      std_srvs::Empty::Response & /*res*/) {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/,
                     std_srvs::Empty::Response & /*res*/) {
    running_ = false;
    return true;
  }

  void WheelEncoderCallback(const mirte_msgs::Encoder::ConstPtr &msg,
                            int joint) {
    if (msg->value < 0) {
      bidirectional = true;
    }
    _wheel_encoder[joint] = msg->value;
    _wheel_encoder_update_time[joint] = msg->header.stamp;
  }

  // Thread and function to restart service clients when the service server has
  // restarted
  std::future<void> reconnect_thread;
  void init_service_clients();
  void start_reconnect();
  std::mutex service_clients_mutex;

  bool bidirectional = false; // assume it is one direction, when receiving any
                              // negative value, it will be set to true
  unsigned int NUM_JOINTS = 2;
}; // class

} // namespace mirte_base_control