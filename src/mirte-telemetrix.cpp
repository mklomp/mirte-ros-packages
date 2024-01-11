#include "mirte-actuators.hpp"
#include "mirte-board.hpp"
#include "mirte-ping.hpp"
#include "mirte-sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>
int main(int argc, char **argv) {
  // Initialize the ROS node
  try {

    ros::init(argc, argv, "mirte_telemetrix");
    TMX tmx("/dev/ttyACM0");
    tmx.sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
    tmx.setScanDelay(10);
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Your code here
    Mirte_Board board(tmx, nh);

    Mirte_Sensors monitor(tmx, nh, board);
    Mirte_Actuators actuators(tmx, nh, board);
    Mirte_Ping ping(tmx, nh, [&]() {
      std::cout << "stop" << std::endl;
      ros::shutdown();
    });
    // Spin the ROS node
    ros::spin();
  } catch (std::exception &e) {
    std::cout << "Exception" << e.what() << std::endl;
  }
  return 0;
}
