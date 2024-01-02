#include "mirte-actuators.hpp"
#include "mirte-board.hpp"
#include "mirte-sensors.hpp"
#include <ros/ros.h>
#include <tmx.hpp>
int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "my_node");
  TMX tmx("/dev/ttyACM0");
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Your code here
  Mirte_Board board(tmx, nh);

  Mirte_Sensors monitor(tmx, nh, board);
  // Spin the ROS node
  ros::spin();

  return 0;
}
