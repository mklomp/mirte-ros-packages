#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca_data.hpp>

PCAData::PCAData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: I2CModuleData(parser, board, name, /*this->get_module_type()*/ parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  // TODO: Temporary new default for address
  if ((!parameters.count("id")) && this->addr == 0xFF) this->addr = 0x41;

  if (unused_keys.erase("frequency")) this->frequency = parameters["frequency"].get<int>();

  // FIXME: Remove
  auto pca_key = parser->build_param_name(get_device_class(), name);

  if (unused_keys.erase("motors")) {
    //TODO: MOTORS REPARSE
    RCLCPP_INFO(logger, "Attempting to find motors [%s]", pca_key.c_str());
    this->motors = PCA_Motor_data::parse_pca_motor_data(parser, board, pca_key);
  }

  if (unused_keys.erase("servos")) {
    //TODO: SERVOS REPARSE
    this->servos = PCA_Servo_data::parse_pca_servo_data(parser, board, pca_key);
  }
}

bool PCAData::check() { return I2CModuleData::check(get_module_type()); }

// std::vector<std::shared_ptr<PCA_data>> PCA_data::parse_pca_data(
//   std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board)
// {
//   std::vector<std::shared_ptr<PCA_data>> pcas;
//   // get modules
//   // get all modules with type==PCA
//   // for each module, parse motors and servos
//   for (auto name : parser->get_params_keys("modules")) {
//     auto mod_key = parser->build_param_name("modules", name);
//     auto mod_config = parser->get_params_name(mod_key);
//     auto mod_keys = parser->get_params_keys(mod_key);
//     if (mod_keys.count("type")) {
//       std::string type = mod_config["type"].get<std::string>();
//       boost::algorithm::to_lower(type);
//       if (type == "pca9685") {
//         auto pca_data = PCA_data::parse_pca_data_single(parser, board, mod_key);
//         if (pca_data->check()) {
//           pcas.push_back(pca_data);
//         }
//       }
//     }
//   }
//   return pcas;
// }
// std::shared_ptr<PCA_data> PCA_data::parse_pca_data_single(
//   std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key)
// {
//   auto pca_config = parser->get_params_name(pca_key);
//   auto pca_keys = parser->get_params_keys(pca_key);
//   PCA_data pca_data;
//   pca_data.name = parser->get_last(pca_key);
//   if (pca_keys.count("id")) {
//     pca_data.addr = pca_config["id"].get<uint8_t>();
//   }
//   if (pca_keys.count("connector")) {
//     auto conn_name = pca_config["connector"].get<std::string>();
//     auto pins = board->resolveConnector(conn_name);
//     pca_data.scl = pins["scl"];
//     pca_data.sda = pins["sda"];
//     boost::algorithm::to_lower(conn_name);
//     pca_data.port = board->resolveI2CPort(pca_data.sda);
//   }
//   if (pca_keys.count("frequency")) {
//     pca_data.frequency = pca_config["frequency"].get<int>();
//   }
//   if (pca_keys.count("motors")) {
//     pca_data.motors = PCA_Motor_data::parse_pca_motor_data(parser, board, pca_key);
//   }
//   if (pca_keys.count("servos")) {
//     pca_data.servos = PCA_Servo_data::parse_pca_servo_data(parser, board, pca_key);
//   }
//   return std::make_shared<PCA_data>(pca_data);
// }

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
        servo_data.pin = board->resolvePin(servo_config["pin"].get<std::string>());
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