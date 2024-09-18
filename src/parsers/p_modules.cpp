#include <mirte_telemetrix_cpp/parsers/p_modules.hpp>

#include <boost/algorithm/string.hpp>
#include <cmath>
#include <mirte_telemetrix_cpp/parsers/actuators.hpp>

std::vector<std::shared_ptr<Hiwonder_bus_data>>
Hiwonder_bus_data::parse_hiwonder_bus_data(std::shared_ptr<Parser> parser,
                                           std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Hiwonder_bus_data>> buses;
  for (auto name : parser->get_params_keys("modules")) {
    auto mod_key = parser->build_param_name("modules", name);
    auto mod_config = parser->get_params_name(mod_key);
    auto mod_keys = parser->get_params_keys(mod_key);
    if (mod_keys.count("type")) {
      std::string type = mod_config["type"].get<std::string>();

      boost::algorithm::to_lower(type);
      std::cout << "found module with type" << type << std::endl;

      if (type == "hiwonder_servo") {
        auto pca_data = Hiwonder_bus_data::parse_single(parser, board, mod_key);
        if (pca_data->check()) {
          buses.push_back(pca_data);
        }
      }
    }
  }
  return buses;
}
std::shared_ptr<Hiwonder_bus_data>
Hiwonder_bus_data::parse_single(std::shared_ptr<Parser> parser,
                                std::shared_ptr<Mirte_Board> board,
                                std::string bus_key) {
  std::cout << "parsing hiwonder Bus: " << bus_key << std::endl;
  auto bus_config = parser->get_params_name(bus_key);
  auto bus_keys = parser->get_params_keys(bus_key);
  Hiwonder_bus_data bus_data;
  bus_data.name = parser->get_last(bus_key);
  std::cout << "parsing hiwonder Bus: " << bus_key << std::endl;
  if (bus_keys.count("uart")) {
    bus_data.uart_port = bus_config["uart"].get<uint8_t>();
    if (bus_data.uart_port > 1) {
      std::cout << "Invalid uart port: " << bus_data.uart_port << std::endl;
      return nullptr;
    }
  }
  std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
  if (bus_keys.count("rx_pin")) {
    bus_data.rx_pin = board->resolvePin(get_string(bus_config["rx_pin"]));
  }
  std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
  if (bus_keys.count("tx_pin")) {
    bus_data.tx_pin = board->resolvePin(get_string(bus_config["tx_pin"]));
  }
  std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;

  if (bus_keys.count("servos")) {
    bus_data.servos =
        Hiwonder_servo_data::parse_hiwonder_servo_data(parser, board, bus_key);
  }
  return std::make_shared<Hiwonder_bus_data>(bus_data);
}

double deg_to_rad(double deg) { return deg * M_PI / 180; }

std::vector<std::shared_ptr<Hiwonder_servo_data>>
Hiwonder_servo_data::parse_hiwonder_servo_data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string bus_key) {
  std::vector<std::shared_ptr<Hiwonder_servo_data>> servos;
  auto bus_config = parser->get_params_name(bus_key);
  auto bus_keys = parser->get_params_keys(bus_key);
  if (bus_keys.count("servos")) {
    auto servos_name = parser->build_param_name(bus_key, "servos");
    auto servos_config = parser->get_params_name(servos_name);
    for (auto servo_key : parser->get_params_keys(servos_name)) {
      auto servo_config = parser->get_params_name(
          parser->build_param_name(servos_name, servo_key));
      auto servo_keys = parser->get_params_keys(
          parser->build_param_name(servos_name, servo_key));
      Hiwonder_servo_data servo_data;
      servo_data.name = servo_key;
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
      if (servo_keys.count("id")) {
        servo_data.id = servo_config["id"].get<uint8_t>();
      }
      if (servo_keys.count("min_angle_out")) {
        std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
        servo_data.min_angle_out = servo_config["min_angle_out"].get<int>();
      } else {
        continue;
      }
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
      if (servo_keys.count("max_angle_out")) {
        servo_data.max_angle_out = servo_config["max_angle_out"].get<int>();
      } else {
        continue;
      }
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
      if (servo_keys.count("home_out")) {
        servo_data.home_out = servo_config["home_out"].get<int>();
      } else {
        continue;
      }
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;

      if (servo_keys.count("invert")) {
        servo_data.min_angle_in = servo_config["invert"].get<bool>();
      }
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
      if (servo_keys.count("frame")) {
        std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
        servo_data.frame_id = servo_config["frame"].get<std::string>();
      } else {
        std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
        servo_data.frame_id = "servo_" + servo_data.name;
      }
      // calculate min_angle_in and max_angle_in
      double diff_min = servo_data.min_angle_out - servo_data.home_out;
      diff_min = diff_min / 100;
      servo_data.min_angle_in = deg_to_rad(diff_min);
      double diff_max = servo_data.max_angle_out - servo_data.home_out;
      diff_max = diff_max / 100;
      servo_data.max_angle_in = deg_to_rad(diff_max);
      if (servo_data.invert) {
        double t = servo_data.min_angle_in;
        servo_data.min_angle_in = -servo_data.max_angle_in;
        servo_data.max_angle_in = -t;
      }
      std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
      std::cout << "Servo: " << servo_data.name
                << " min_angle_in: " << servo_data.min_angle_in
                << " max_angle_in: " << servo_data.max_angle_in << std::endl;
      if (servo_data.check()) {
        servos.push_back(std::make_shared<Hiwonder_servo_data>(servo_data));
      }
    }
  }
  return servos;
}

std::vector<std::shared_ptr<INA226_data>>
INA226_data::parse_ina226_data(std::shared_ptr<Parser> parser,
                               std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<INA226_data>> inas;
  for (auto name : parser->get_params_keys("modules")) {
    auto mod_key = parser->build_param_name("modules", name);
    auto mod_config = parser->get_params_name(mod_key);
    auto mod_keys = parser->get_params_keys(mod_key);
    if (mod_keys.count("type")) {
      std::string type = mod_config["type"].get<std::string>();

      boost::algorithm::to_lower(type);
      if (type == "ina226") {
        std::cout << "Found one ina" << mod_key << std::endl;
        auto ina_data =
            INA226_data::parse_ina226_data_single(parser, board, mod_key);
        if (ina_data->check()) {
          inas.push_back(ina_data);
        }
      }
    }
  }
  return inas;
}

std::shared_ptr<INA226_data>
INA226_data::parse_ina226_data_single(std::shared_ptr<Parser> parser,
                                      std::shared_ptr<Mirte_Board> board,
                                      std::string ina_key) {
  auto ina_config = parser->get_params_name(ina_key);
  auto ina_keys = parser->get_params_keys(ina_key);
  INA226_data ina_data;
  ina_data.name = parser->get_last(ina_key);
  if (ina_keys.count("addr")) {
    ina_data.addr = ina_config["addr"].get<uint8_t>();
  }
  if (ina_keys.count("connector")) {
    auto conn_name = ina_config["connector"].get<std::string>();
    auto pins = board->resolveConnector(conn_name);
    ina_data.scl = pins["scl"];
    ina_data.sda = pins["sda"];
    boost::algorithm::to_lower(conn_name);
    ina_data.port = board->resolveI2CPort(ina_data.sda);
  }
  if (ina_keys.count("max_current")) {
    ina_data.max_current = ina_config["max_current"].get<float>();
  }
  if (ina_keys.count("max_voltage")) {
    ina_data.max_voltage = ina_config["max_voltage"].get<float>();
  }
  if (ina_keys.count("min_voltage")) {
    ina_data.min_voltage = ina_config["min_voltage"].get<float>();
  }

  return std::make_shared<INA226_data>(ina_data);
}