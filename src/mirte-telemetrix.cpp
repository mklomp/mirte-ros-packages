#include "mirte-actuators.hpp"
#include "mirte-board.hpp"
#include "mirte-sensors.hpp"
#include <ros/ros.h>
#include <tmx.hpp>
int main(int argc, char **argv) {
  // Initialize the ROS node
  try {

    ros::init(argc, argv, "my_node");
    TMX tmx("/dev/ttyACM0");
    tmx.sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Your code here
    Mirte_Board board(tmx, nh);

    Mirte_Sensors monitor(tmx, nh, board);
    // Spin the ROS node
    ros::spin();
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
