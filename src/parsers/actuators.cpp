#include <parsers/actuators.hpp>


std::vector<std::shared_ptr<Servo_data>> parse_servo_data(std::shared_ptr<rclcpp::Node> nh)
{
  std::vector<std::shared_ptr<Servo_data>> servos;
  rclcpp::Parameter servos_config;
  
  // nh->get_parameter("/mirte/servo", servos_config);
  // for (auto &servo_it : servos_config) {
  //   std::cout << servo_it.first << std::endl;
  //   rclcpp::Parameter servo_config = servo_it.second;
  //   ROS_ASSERT(servo_config.get_type() == rclcpp::ParameterType::PARAMETER_STRUCT);
  //   std::string name = servo_it.first;
  //   std::string connector = servo_config["connector"];
  //   pin_t pin = servo_config["pin"];
  //   int min_pulse = servo_config["min_pulse"];
  //   int max_pulse = servo_config["max_pulse"];
  // //   servos.push_back(std::make_shared<Servo_data>(name, pin, connector, min_pulse, max_pulse));
  // }
//   rclcpp::Parameter motors_config;
// nh.getParam("/mirte/motor", motors_config);
// for (auto &motor_it : motors_config) {
//   std::cout << motor_it.first << std::endl;
//   rclcpp::Parameter motor_config = motor_it.second;
//   ROS_ASSERT(motor_config.getType() == rclcpp::Parameter::TypeStruct);
//   std::string type = motor_config["type"];
//   std::string name = motor_it.first;
//   std::vector<uint8_t> pins = board.resolvePins(motor_config["pins"]);
//   if (type == "pp") {
//     motors.push_back(new PPMotor( nh, tmx, board, pins, name));
//   } else if (type == "dp") {
//     motors.push_back(new DPMotor( nh, tmx, board, pins, name));
//   } else {
//     ROS_ERROR("Unknown motor type: %s", type.c_str());
//   }
// }

  return servos;
}
