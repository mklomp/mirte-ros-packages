#include "parsers/p_sensors.hpp"

std::vector<std::shared_ptr<Sonar_data>>
Sonar_data::parse_sonar_data(std::shared_ptr<Parser> parser,
                             std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Sonar_data>> sonars;
  for (auto name : parser->get_params_keys("distance")) {
    Sonar_data sonar_data;
    sonar_data.name = name;
    auto sonar_key = parser->build_param_name("distance", name);
    auto sonar_config = parser->get_params_name(sonar_key);
    auto sonar_keys = parser->get_params_keys(sonar_key);
    if (sonar_config.count("connector")) {
      std::cout << "connector" << std::endl;
      std::string connector = get_string(sonar_config["connector"]);
      auto pins = board->resolveConnector(connector);
      sonar_data.trigger = pins["trigger"];
      sonar_data.echo = pins["echo"];

    } else if (sonar_keys.count("pins")) {
      auto pins_config =
          parser->get_params_name(parser->build_param_name(sonar_key, "pins"));
      for (auto pin_key : parser->get_params_keys(
               parser->build_param_name(sonar_key, "pins"))) {
        if ("trigger" == pin_key) {
          sonar_data.trigger =
              board->resolvePin(get_string(pins_config[pin_key]));
        } else if ("echo" == pin_key) {
          sonar_data.echo = board->resolvePin(get_string(pins_config[pin_key]));
        }
      }
    }
    if (sonar_data.check()) {
      sonars.push_back(std::make_shared<Sonar_data>(sonar_data));
    }
  }
  return sonars;
}

std::vector<std::shared_ptr<Intensity_data>>
Intensity_data::parse_intensity_data(std::shared_ptr<Parser> parser,
                                     std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Intensity_data>> irs;
  for (auto name : parser->get_params_keys("intensity")) {
    Intensity_data intensity_data;
    intensity_data.name = name;
    auto intensity_key = parser->build_param_name("intensity", name);
    auto intensity_config = parser->get_params_name(intensity_key);
    auto intensity_keys = parser->get_params_keys(intensity_key);
    if (intensity_config.count("connector")) {
      std::cout << "connector" << std::endl;
      std::string connector = get_string(intensity_config["connector"]);
      auto pins = board->resolveConnector(connector);
      if (pins.count("analog")) {
        intensity_data.a_pin = pins["analog"];
      }
      if (pins.count("digital")) {
        intensity_data.d_pin = pins["digital"];
      }
    } else if (intensity_keys.count("pins")) {
      auto pins_config = parser->get_params_name(
          parser->build_param_name(intensity_key, "pins"));
      for (auto pin_key : parser->get_params_keys(
               parser->build_param_name(intensity_key, "pins"))) {
        if ("analog" == pin_key) {
          intensity_data.a_pin =
              board->resolvePin(get_string(pins_config[pin_key]));
        } else if ("digital" == pin_key) {
          intensity_data.d_pin =
              board->resolvePin(get_string(pins_config[pin_key]));
        }
      }
    }
    if (intensity_data.check()) {
      irs.push_back(std::make_shared<Intensity_data>(intensity_data));
    }
  }
  return irs;
}

std::vector<std::shared_ptr<Keypad_data>>
Keypad_data::parse_keypad_data(std::shared_ptr<Parser> parser,
                               std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Keypad_data>> out;
  for (auto name : parser->get_params_keys("keypad")) {
    Keypad_data data;
    data.name = name;
    auto key = parser->build_param_name("keypad", name);
    auto config = parser->get_params_name(key);
    auto keys = parser->get_params_keys(key);
    if (config.count("connector")) {
      std::cout << "connector" << std::endl;
      std::string connector = get_string(config["connector"]);
      auto pins = board->resolveConnector(connector);
      data.pin = pins["pin"];

    } else if (keys.count("pins")) {
      auto pins_config =
          parser->get_params_name(parser->build_param_name(key, "pins"));
      for (auto pin_key :
           parser->get_params_keys(parser->build_param_name(key, "pins"))) {
        if ("pin" == pin_key) {
          data.pin = board->resolvePin(get_string(pins_config[pin_key]));
        }
      }
    }
    if (data.check()) {
      out.push_back(std::make_shared<Keypad_data>(data));
    }
  }
  return out;
}

std::vector<std::shared_ptr<Encoder_data>>
Encoder_data::parse_encoder_data(std::shared_ptr<Parser> parser,
                                 std::shared_ptr<Mirte_Board> board) {
  std::vector<std::shared_ptr<Encoder_data>> encoders;
  for (auto name : parser->get_params_keys("encoder")) {
    Encoder_data encoder_data;
    encoder_data.name = name;
    auto encoder_key = parser->build_param_name("encoder", name);
    auto encoder_config = parser->get_params_name(encoder_key);
    auto encoder_keys = parser->get_params_keys(encoder_key);
    if (encoder_config.count("connector")) {
      std::cout << "connector" << std::endl;
      std::string connector = get_string(encoder_config["connector"]);
      auto pins = board->resolveConnector(connector);
      encoder_data.pinA = pins["pinA"];
      encoder_data.pinB = pins["pinB"];

    } else if (encoder_keys.count("pins")) {
      auto pins_config = parser->get_params_name(
          parser->build_param_name(encoder_key, "pins"));
      for (auto pin_key : parser->get_params_keys(
               parser->build_param_name(encoder_key, "pins"))) {
        if ("A" == pin_key) {
          encoder_data.pinA =
              board->resolvePin(get_string(pins_config[pin_key]));
        } else if ("B" == pin_key) {
          encoder_data.pinB =
              board->resolvePin(get_string(pins_config[pin_key]));
        } else if ("pin" == pin_key) { // single pin encoder
          encoder_data.pinA =
              board->resolvePin(get_string(pins_config[pin_key]));
          encoder_data.pinB = (pin_t)-1;
        }
      }
    }
    if (encoder_data.check()) {
      encoders.push_back(std::make_shared<Encoder_data>(encoder_data));
    }
  }
  return encoders;
}