//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
// Roughlt based on:
// https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/include/my_robot_hw_interface.h
// https://github.com/PickNikRobotics/ros_control_boilerplate
// https://github.com/DeborggraeveR/ampru
// 

// https://github.com/resibots/dynamixel_control_hw/blob/master/include/dynamixel_control_hw/hardware_interface.hpp
// https://github.com/FRC900/2018RobotCode/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frcrobot_hw_interface.h



#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
// ostringstream
#include <sstream>
#include <algorithm>
#include <cmath>

const unsigned int NUM_JOINTS = 2;

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface();

  std_msgs::Int32 prev_left;
  std_msgs::Int32 prev_right;

  /*
   *
   */
  void write() {
    //255 pwm = 90hz -> 90/sec -> ca 2 omwenteling /s -> ca 40 cm /s -> 0.4m/s


    double diff_speed_left = angularToLinear(cmd[0]);  // TODO: angularToLinear not needed since we can do everythign in agular space
    double diff_speed_right = angularToLinear(cmd[1]);

    std_msgs::Int32 left_pwm;
    std_msgs::Int32 right_pwm;
    left_pwm.data = (int)(diff_speed_left * 600);
    right_pwm.data = (int)(diff_speed_right * 600);

    double voltage_left = std::min(std::max(-5.0,diff_speed_left * 50), 5.0) ;
    double voltage_right = std::min(std::max(-5.0,diff_speed_right * 50), 5.0) ;

    int left_factor;
    nh.param("left_factor", left_factor, 1);
    int right_factor;
    nh.param("right_factor", right_factor, 1);

    _last_wheel_cmd_direction[0] = voltage_left / std::abs(voltage_left);
    _last_wheel_cmd_direction[1] = voltage_right / std::abs(voltage_right);

    if (left_pwm.data != prev_left.data){
	left_wheel_command_pub_.publish(left_pwm);
        prev_left = left_pwm;
    }
    if (right_pwm.data != prev_right.data){
	right_wheel_command_pub_.publish(right_pwm);
        prev_right = right_pwm;
    }

    std::ostringstream os;
    os << "Wheel cmd: " << cmd[0] << "  "  << voltage_left << "     "  << voltage_right;
    ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << os.str()); 
 }

  /**
   * Reading encoder values and setting position and velocity of enconders 
   */
  void read(const ros::Duration &period) {

    double wheelRadius = 0.065;
    double meterPerEncoderTick = wheelRadius * 3.1415926 / 40; 
    double distance_left = _wheel_encoder[0] * meterPerEncoderTick;
    double distance_right = _wheel_encoder[1] * meterPerEncoderTick;


    pos[0] += linearToAngular(distance_left) * _last_wheel_cmd_direction[0];
    vel[0] = linearToAngular(distance_left) / period.toSec() * _last_wheel_cmd_direction[0];
    pos[1] += linearToAngular(distance_right) * _last_wheel_cmd_direction[1];
    vel[1] = linearToAngular(distance_right) / period.toSec() * _last_wheel_cmd_direction[0];

    std::ostringstream os;
    os << "Wheel velocitu: " << _wheel_encoder[0] / period.toSec() << "     "  << _wheel_encoder[1] / period.toSec() << "      "  << period.toSec();
    ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << os.str());    

    _wheel_encoder[0] = 0;
    _wheel_encoder[1] = 0;
  }

  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];
  int _wheel_encoder[NUM_JOINTS];

  int _last_wheel_cmd_direction[NUM_JOINTS];


  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber left_wheel_angle_sub_;
  ros::Subscriber right_wheel_angle_sub_;
  ros::Publisher left_wheel_vel_pub_;
  ros::Publisher right_wheel_vel_pub_;

  ros::Subscriber left_wheel_encoder_sub_;
  ros::Subscriber right_wheel_encoder_sub_;
  ros::Publisher left_wheel_command_pub_;
  ros::Publisher right_wheel_command_pub_;


  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  { 
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }


  void leftWheelEncoderCallback(const std_msgs::Int32& msg) {
    _wheel_encoder[0]++;
  }

  void rightWheelEncoderCallback(const std_msgs::Int32& msg) {
    _wheel_encoder[1]++;
  }


  void leftWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[0] = msg.data;
  }

  void rightWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[1] = msg.data;
  }

  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
	double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
	if (speed > _max_speed) {
		diff_speed_left *= _max_speed / speed;
		diff_speed_right *= _max_speed / speed;
	}
  }

double linearToAngular(const double &travel) const
{
    return travel / 0.065;
}

double angularToLinear(const double &angle) const
{
    return angle * 0.065;
}


};  // class

MyRobotHWInterface::MyRobotHWInterface()
: running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MyRobotHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback, this)) 
  {
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.064);
    private_nh.param<double>("max_speed", _max_speed, 1.0);
  
    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
      jnt_vel_interface.registerHandle(vel_handle);

      _wheel_encoder[i] = 0;
      _last_wheel_cmd_direction[i] = 0;
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

	// Initialize publishers and subscribers
	left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("my_robot/left_wheel_vel", 1);
	right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("my_robot/right_wheel_vel", 1);

	left_wheel_angle_sub_ = nh.subscribe("my_robot/left_wheel_angle", 1, &MyRobotHWInterface::leftWheelAngleCallback, this);
	right_wheel_angle_sub_ = nh.subscribe("my_robot/right_wheel_angle", 1, &MyRobotHWInterface::rightWheelAngleCallback, this);

	left_wheel_encoder_sub_ = nh.subscribe("left_encoder", 1, &MyRobotHWInterface::leftWheelEncoderCallback, this);
	right_wheel_encoder_sub_ = nh.subscribe("right_encoder", 1, &MyRobotHWInterface::rightWheelEncoderCallback, this);

	left_wheel_command_pub_ = nh.advertise<std_msgs::Int32>("left_pwm", 1);
	right_wheel_command_pub_ = nh.advertise<std_msgs::Int32>("right_pwm", 1);
}
