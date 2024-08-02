#include <parsers/p_modules.hpp>

#include <boost/algorithm/string.hpp>
#include <parsers/actuators.hpp>
#include <cmath>
std::vector<std::shared_ptr<PCA_data>> PCA_data::parse_pca_data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<PCA_data>> pcas;
        // get modules
        // get all modules with type==PCA
        // for each module, parse motors and servos
        for(auto name: parser->get_params_keys("modules")) {
                auto mod_key = parser->build_param_name("modules", name);
                auto mod_config = parser->get_params_name(mod_key);
                auto mod_keys = parser->get_params_keys(mod_key);
                if(mod_keys.count("type")) {
                        std::string type = mod_config["type"].get<std::string>();
                        boost::algorithm::to_lower(type);
                        if(type == "pca9685") {
                                auto pca_data = PCA_data::parse_pca_data_single(parser, board, mod_key);
                                if(pca_data->check()) {
                                        pcas.push_back(pca_data);
                                }
                        }
                }

        }
        return pcas;
    }
    std::shared_ptr<PCA_data> PCA_data::parse_pca_data_single(
        std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key) {
        auto pca_config = parser->get_params_name(pca_key);
        auto pca_keys = parser->get_params_keys(pca_key);
        PCA_data pca_data;
        pca_data.name = pca_key;
        if(pca_keys.count("addr")) {
            pca_data.addr = pca_config["addr"].get<uint8_t>();
        }
        if(pca_keys.count("connector")) {
                auto conn_name = pca_config["connector"].get<std::string>();
                auto pins = board->resolveConnector(conn_name);
                pca_data.scl = pins["scl"];
                pca_data.sda = pins["sda"];
                boost::algorithm::to_lower(conn_name);
            pca_data.port = conn_name == ("i2c1") ? 1 : 2;
        }
        if(pca_keys.count("frequency")) {
            pca_data.frequency = pca_config["frequency"].get<int>();
        }
        if(pca_keys.count("motors")) {
            pca_data.motors = PCA_Motor_data::parse_pca_motor_data(parser, board, pca_key);
        }
                if(pca_keys.count("servos")) {
                pca_data.servos = PCA_Servo_data::parse_pca_servo_data(parser, board, pca_key);
                }
                return std::make_shared<PCA_data>(pca_data);
        }


std::vector<std::shared_ptr<PCA_Motor_data>> PCA_Motor_data::parse_pca_motor_data(
        std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key) {
        std::vector<std::shared_ptr<PCA_Motor_data>> motors;
        auto pca_config = parser->get_params_name(pca_key);
        auto pca_keys = parser->get_params_keys(pca_key);
        if(pca_keys.count("motors")) {
                auto motors_name = parser->build_param_name(pca_key, "motors");

                auto motors_config = parser->get_params_name(motors_name);
                for(auto motor_key: parser->get_params_keys(motors_name)) {
                        auto motor_config = parser->get_params_name(parser->build_param_name(motors_name, motor_key));
                        auto motor_keys = parser->get_params_keys(parser->build_param_name(motors_name, motor_key));
                        PCA_Motor_data motor_data;
                        motor_data.name = motor_key;
                        if(motor_keys.count("pin_A")) {
                                motor_data.pinA = motor_config["pin_A"].get<pin_t>();
                        }
                        if(motor_keys.count("pin_B")) {
                                motor_data.pinB = motor_config["pin_B"].get<pin_t>();
                        }
                        if(motor_keys.count("invert")) {
                                motor_data.invert = motor_config["invert"].get<bool>();
                        }
                        if(motor_data.check()) {
                          std::cout << "added pca motor " << motor_data.name << std::endl;
                                motors.push_back(std::make_shared<PCA_Motor_data>(motor_data));
                        }
                }
        }
        return motors;
        }


std::vector<std::shared_ptr<PCA_Servo_data>> PCA_Servo_data::parse_pca_servo_data(
        std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string pca_key) {
        std::vector<std::shared_ptr<PCA_Servo_data>> servos;
        auto pca_config = parser->get_params_name(pca_key);
        auto pca_keys = parser->get_params_keys(pca_key);
        if(pca_keys.count("servos")) {
                auto servos_name = parser->build_param_name(pca_key, "servos");
                auto servos_config = parser->get_params_name(servos_name);
                for(auto servo_key: parser->get_params_keys(servos_name)) {
                        auto servo_config = parser->get_params_name(parser->build_param_name(servos_name, servo_key));
                        auto servo_keys = parser->get_params_keys(parser->build_param_name(servos_name, servo_key));
                        PCA_Servo_data servo_data;
                        servo_data.name = servo_key;
                        if(servo_keys.count("pin")) {
                                servo_data.pin = board->resolvePin(servo_config["pin"].get<std::string>());
                        }
                        if(servo_keys.count("min_pulse")) {
                                servo_data.min_pulse = servo_config["min_pulse"].get<int>();
                        }
                        if(servo_keys.count("max_pulse")) {
                                servo_data.max_pulse = servo_config["max_pulse"].get<int>();
                        }
                        if(servo_data.check()) {
                                servos.push_back(std::make_shared<PCA_Servo_data>(servo_data));
                        }
                }
        }
        return servos;
}


std::vector<std::shared_ptr<Hiwonder_bus_data>> Hiwonder_bus_data::parse_hiwonder_bus_data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Hiwonder_bus_data>> buses;
  for (auto name : parser->get_params_keys("bus")) {
    auto bus_key = parser->build_param_name("bus", name);
    auto bus_config = parser->get_params_name(bus_key);
    auto bus_keys = parser->get_params_keys(bus_key);
    Hiwonder_bus_data bus_data;
    bus_data.name = name;
    if(bus_keys.count("uart")) {
      bus_data.uart_port = bus_config["uart"].get<uint8_t>();
      if(bus_data.uart_port > 1) {
        std::cout << "Invalid uart port: " << bus_data.uart_port << std::endl;
        continue;
      }
    }
    if(bus_keys.count("rx_pin")) {
      bus_data.rx_pin = board->resolvePin(bus_config["rx_pin"].get<std::string>());
    }
    if(bus_keys.count("tx_pin")) {
      bus_data.tx_pin = board->resolvePin(bus_config["tx_pin"].get<std::string>());
    }

    if (bus_keys.count("servos")) {
      bus_data.servos = Hiwonder_servo_data::parse_hiwonder_servo_data(parser, board, bus_key);
    }
     if (bus_data.check()) {
      buses.push_back(std::make_shared<Hiwonder_bus_data>(bus_data));
    }
  }
  return buses;
}

double deg_to_rad(double deg) {
  return deg * M_PI / 180;
}

std::vector<std::shared_ptr<Hiwonder_servo_data>> Hiwonder_servo_data::parse_hiwonder_servo_data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string bus_key) {
  std::vector<std::shared_ptr<Hiwonder_servo_data>> servos;
  auto bus_config = parser->get_params_name(bus_key);
  auto bus_keys = parser->get_params_keys(bus_key);
  if (bus_keys.count("servos")) {
    auto servos_name = parser->build_param_name(bus_key, "servos");
    auto servos_config = parser->get_params_name(servos_name);
    for (auto servo_key : parser->get_params_keys(servos_name)) {
      auto servo_config = parser->get_params_name(parser->build_param_name(servos_name, servo_key));
      auto servo_keys = parser->get_params_keys(parser->build_param_name(servos_name, servo_key));
      Hiwonder_servo_data servo_data;
      servo_data.name = servo_key;
      if (servo_keys.count("id")) {
        servo_data.id =  servo_config["id"].get<uint8_t>();
      }
      if (servo_keys.count("min_angle_out")) {
        servo_data.min_angle_out = servo_config["min_angle_out"].get<int>();
      } else { continue;}
      if (servo_keys.count("max_angle_out")) {
        servo_data.max_angle_out = servo_config["max_angle_out"].get<int>();
      } else { continue;}
      if (servo_keys.count("home_out")) {
        servo_data.home_out = servo_config["home_out"].get<int>();
      } else { continue;}

      if(servo_keys.count("invert")) {
        servo_data.min_angle_in = servo_config["invert"].get<bool>();
      }
      if(servo_keys.count("frame")) {
        servo_data.frame_id = servo_config["frame"].get<std::string>();
      } else {
        servo_data.frame_id = "servo_"+ servo_data.name;
      }
      // calculate min_angle_in and max_angle_in
        double diff_min = servo_data.min_angle_out - servo_data.home_out;
        diff_min = diff_min / 100;
        servo_data.min_angle_in = deg_to_rad(diff_min);
        double diff_max = servo_data.max_angle_out - servo_data.home_out;
        diff_max = diff_max / 100;
        servo_data.max_angle_in = deg_to_rad(diff_max); 
        if(servo_data.invert) {
          double t = servo_data.min_angle_in;
          servo_data.min_angle_in = -servo_data.max_angle_in;
          servo_data.max_angle_in = -t;
        }
        std::cout << "Servo: " << servo_data.name << " min_angle_in: " << servo_data.min_angle_in << " max_angle_in: " << servo_data.max_angle_in << std::endl;
      if (servo_data.check()) {
        servos.push_back(std::make_shared<Hiwonder_servo_data>(servo_data));
      }
    }
  }
  return servos;
}







#if 0
std::vector<std::shared_ptr<Servo_data>>
Servo_data::parse_servo_data(std::shared_ptr<Parser> parser,
                             std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Servo_data>> servos;
  for (auto name : parser->get_params_keys("servo")) {
    std::cout << name << std::endl;
    auto servo_key = parser->build_param_name("servo", name);
    auto servos_config = parser->get_params_name(servo_key);
    auto servo_keys = parser->get_params_keys(servo_key);
    Servo_data servo_data;
    servo_data.name = name;
    if (servos_config.count("connector")) {
      std::cout << "connector" << std::endl;
      std::string connector = servos_config["connector"].get<std::string>();
      auto pins = board->resolveConnector(connector);
      servo_data.pin = pins["pin"];
    }
    pin_t pin = -1;
    if (servo_keys.count("pins")) {
      std::cout << "pin" << std::endl;
      // pins:
      //   pin: 1
      servo_data.pin =
          board->resolvePin(parser
                                ->get_params_name(parser->build_param_name(
                                    servo_key, "pins"))["pin"]
                                .get<std::string>());
    }

    if (servo_keys.count("min_pulse")) {
      servo_data.min_pulse = servos_config["min_pulse"].get<int>();
    }
    if (servo_keys.count("max_pulse")) {
      servo_data.max_pulse = servos_config["max_pulse"].get<int>();
    }
    if (servo_data.check()) {
      servos.push_back(std::make_shared<Servo_data>(servo_data));
    }
  }
  return servos;
}

std::vector<std::shared_ptr<Motor_data>>
Motor_data::parse_motor_data(std::shared_ptr<Parser> parser,
                             std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Motor_data>> motors;
  for (auto name : parser->get_params_keys("motor")) {
    auto motor_key = parser->build_param_name("motor", name);
    auto motor_config = parser->get_params_name(motor_key);
    auto motor_keys = parser->get_params_keys(motor_key);

    Motor_data motor_data;
    motor_data.name = name;
    if (motor_keys.count("pins")) {
      std::cout << name << "pins" << std::endl;
      auto pins_config =
          parser->get_params_name(parser->build_param_name(motor_key, "pins"));
      for (auto pin_key : parser->get_params_keys(
               parser->build_param_name(motor_key, "pins"))) {
        boost::algorithm::to_lower(pin_key);
        if ("p1" == pin_key) {
          motor_data.P1 =
              board->resolvePin(pins_config[pin_key].get<std::string>());
        }
        if ("p2" == pin_key) {
          motor_data.P2 =
              board->resolvePin(pins_config[pin_key].get<std::string>());
        }
        if ("p1" == pin_key) {
          motor_data.D1 =
              board->resolvePin(pins_config[pin_key].get<std::string>());
        }
        if ("p2" == pin_key) {
          motor_data.D2 =
              board->resolvePin(pins_config[pin_key].get<std::string>());
        }
      }
    } else if (motor_keys.count("connector")) {
      std::string connector = motor_config["connector"].get<std::string>();
      auto conn_pins = board->resolveConnector(connector);
      motor_data.P1 = conn_pins["P1"];
      motor_data.P2 = conn_pins["P2"];
      motor_data.D1 = conn_pins["D1"];
      motor_data.D2 = conn_pins["D2"];
    }
    if (motor_keys.count("type")) {
      std::string type = motor_config["type"].get<std::string>();
      boost::algorithm::to_lower(type);
      if (type == "pp") {
        motor_data.type = Motor_data::Motor_type::PP;
      } else if (type == "dp") {
        motor_data.type = Motor_data::Motor_type::DP;
      } else if (type == "ddp") {
        motor_data.type = Motor_data::Motor_type::DDP;
      } else {
        std::cout << "Unknown motor type: " << type << std::endl;
      }
    } else {
      motor_data.type = Motor_data::Motor_type::PP;
    }
    if (motor_keys.count("inverted")) {
      motor_data.inverted = motor_config["inverted"].get<bool>();
    }
    if (motor_data.check()) {
      motors.push_back(std::make_shared<Motor_data>(motor_data));
    }
  }
  return motors;
}

// TODO: oled and module actuators
#endif