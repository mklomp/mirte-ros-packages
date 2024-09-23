#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca_data.hpp>

PCAData::PCAData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: I2CModuleData(parser, board, name, parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF) this->addr = 0x41;

  if (unused_keys.erase("frequency")) this->frequency = parameters["frequency"].get<int>();

  // FIXME: Remove
  auto pca_key = parser->build_param_name(get_device_class(), name);

  if (unused_keys.erase("motors")) {
    RCLCPP_DEBUG(logger, "Attempting to find PCA motors [%s]", pca_key.c_str());
    //TODO: MOTORS REPARSE
    this->motors = PCA_Motor_data::parse_pca_motor_data(parser, board, pca_key);
  }

  if (unused_keys.erase("servos")) {
    RCLCPP_DEBUG(logger, "Attempting to find PCA servos [%s]", pca_key.c_str());
    //TODO: SERVOS REPARSE
    this->servos = PCA_Servo_data::parse_pca_servo_data(parser, board, pca_key);
  }
}

bool PCAData::check() { return I2CModuleData::check(get_module_type()); }

std::vector<std::shared_ptr<PCA_Motor_data>> PCA_Motor_data::parse_pca_motor_data(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key)
{
  std::vector<std::shared_ptr<PCA_Motor_data>> motors;
  auto pca_config = parser->get_params_name(pca_key);
  auto pca_keys = parser->get_params_keys(pca_key);
  if (pca_keys.count("motors")) {
    auto motors_name = parser->build_param_name(pca_key, "motors");

    auto motors_config = parser->get_params_name(motors_name);
    for (auto motor_key : parser->get_params_keys(motors_name)) {
      auto motor_config = parser->get_params_name(parser->build_param_name(motors_name, motor_key));
      auto motor_keys = parser->get_params_keys(parser->build_param_name(motors_name, motor_key));
      PCA_Motor_data motor_data;
      motor_data.name = motor_key;
      if (motor_keys.count("pin_A")) {
        motor_data.pinA = motor_config["pin_A"].get<pin_t>();
      }
      if (motor_keys.count("pin_B")) {
        motor_data.pinB = motor_config["pin_B"].get<pin_t>();
      }
      if (motor_keys.count("invert")) {
        motor_data.invert = motor_config["invert"].get<bool>();
      }
      if (motor_data.check()) {
        std::cout << "added pca motor " << motor_data.name << std::endl;
        motors.push_back(std::make_shared<PCA_Motor_data>(motor_data));
      }
    }
  }
  return motors;
}

std::vector<std::shared_ptr<PCA_Servo_data>> PCA_Servo_data::parse_pca_servo_data(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key)
{
  std::vector<std::shared_ptr<PCA_Servo_data>> servos;
  auto pca_config = parser->get_params_name(pca_key);
  auto pca_keys = parser->get_params_keys(pca_key);
  if (pca_keys.count("servos")) {
    auto servos_name = parser->build_param_name(pca_key, "servos");
    auto servos_config = parser->get_params_name(servos_name);
    for (auto servo_key : parser->get_params_keys(servos_name)) {
      auto servo_config = parser->get_params_name(parser->build_param_name(servos_name, servo_key));
      auto servo_keys = parser->get_params_keys(parser->build_param_name(servos_name, servo_key));
      PCA_Servo_data servo_data;
      servo_data.name = servo_key;
      if (servo_keys.count("pin")) {
        servo_data.pin = servo_config["pin"].get<pin_t>();
      }
      if (servo_keys.count("min_pulse")) {
        servo_data.min_pulse = servo_config["min_pulse"].get<int>();
      }
      if (servo_keys.count("max_pulse")) {
        servo_data.max_pulse = servo_config["max_pulse"].get<int>();
      }
      if (servo_data.check()) {
        servos.push_back(std::make_shared<PCA_Servo_data>(servo_data));
      }
    }
  }
  return servos;
}