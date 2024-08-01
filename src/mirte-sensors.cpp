#include "mirte-sensors.hpp"
#include "parsers/sensors.hpp"

Mirte_Sensors::Mirte_Sensors(std::shared_ptr<rclcpp::Node> nh,
                             std::shared_ptr<TMX> tmx,
                             std::shared_ptr<Mirte_Board> board,
                             std::shared_ptr<Parser> parser) {
  this->tmx = tmx;
  this->nh = nh;
  this->board = board;
  auto keypads = KeypadMonitor::get_keypad_monitors(nh, tmx, board, parser);
  this->sensors.insert(this->sensors.end(), keypads.begin(), keypads.end());
  std::cout << " start sonar" << std::endl;
  auto sonars = SonarMonitor::get_sonar_monitors(nh, tmx, board, parser);
  std::cout << " done sonar" << std::endl;
  this->sensors.insert(this->sensors.end(), sonars.begin(), sonars.end());
  auto irs = IntensityMonitor::get_intensity_monitors(nh, tmx, board, parser);
  this->sensors.insert(this->sensors.end(), irs.begin(), irs.end());
  this->timer = nh->create_wall_timer(
      std::chrono::milliseconds(1000 / parser->get_frequency()),
      std::bind(&Mirte_Sensors::publish, this));

  this->pin_service = nh->create_service<mirte_msgs::srv::GetPinValue>(
      "/mirte/get_pin_value",
      std::bind(&Mirte_Sensors::pin_callback, this, std::placeholders::_1,
                std::placeholders::_2));
}

bool Mirte_Sensors::pin_callback(
    const std::shared_ptr<mirte_msgs::srv::GetPinValue::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetPinValue::Response> res) {
  // calculate time for this function
  auto start2 = std::chrono::system_clock::now();
  bool is_digital = starts_with(req->type, "d") || starts_with(req->type, "D");
  auto pin = this->board->resolvePin(req->pin);
  auto done = false;
  bool has_digital_cb = false;
  bool has_analog_cb = false;
  if (this->pin_map.count(pin)) {
    const auto [type, value, ana_cb, dig_cb] = this->pin_map[pin];
    has_analog_cb = ana_cb;
    has_digital_cb = dig_cb;
    if ((type == PIN_USE::DIGITAL_IN && is_digital) ||
        (type == PIN_USE::ANALOG_IN && !is_digital)) {
      res->data = value;
      // cout time it took
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start2;
      std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
      return true;
    }
  }
  if (is_digital) {
    this->pin_map[pin] = {PIN_USE::DIGITAL_IN, -1, has_analog_cb, true};

    this->tmx->setPinMode(pin, TMX::PIN_MODES::DIGITAL_INPUT, true);
    if (!has_digital_cb) {
      std::cout << "add digital callback" << std::endl;
      this->tmx->add_digital_callback(pin, [this](auto pin, auto value) {
        const auto [type, old_value, has_analog, has_digital] =
            this->pin_map[pin];
        if (type == PIN_USE::DIGITAL_IN && old_value != value) {
          this->pin_map[pin] = {type, value, has_analog, has_digital};
        }
      });
    }
  } else {
    this->pin_map[pin] = {PIN_USE::ANALOG_IN, -1, true, has_digital_cb};

    this->tmx->setPinMode(pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
    if (!has_analog_cb) {
      std::cout << "add a callback" << std::endl;

      this->tmx->add_analog_callback(pin, [this](auto pin, auto value) {
        const auto [type, old_value, has_analog, has_digital] =
            this->pin_map[pin];
        if (type == PIN_USE::ANALOG_IN && old_value != value) {
          this->pin_map[pin] = {type, value, has_analog, has_digital};
        }
      });
    }
  }

  // while time less than 5s, after that its probably not going to change
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < std::chrono::seconds(5)) {
    // if (this->pin_map.count(pin)) {
    const auto [type, value, has_analog, has_digital] = this->pin_map[pin];
    if (value != -1) {
      res->data = value;

      // cout time it took
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start2;
      std::cout << "321elapsed time: " << elapsed_seconds.count() << "s\n";
      return true;
    }

    // sleep for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  res->data = -1;

  return false;
}

void Mirte_Sensors::publish() {
  for (auto sensor : this->sensors) {
    sensor->publish();
  }
}

void Mirte_Sensors::stop() {
  if (!this->timer->is_canceled()) {
    this->timer->cancel();
  }
  for (auto sensor : this->sensors) {
    sensor->stop();
  }
}

std::vector<std::shared_ptr<KeypadMonitor>> KeypadMonitor::get_keypad_monitors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<KeypadMonitor>> sensors;
  auto keypads = Keypad_data::parse_keypad_data(parser, board);
  for (auto keypad : keypads) {
    sensors.push_back(std::make_shared<KeypadMonitor>(nh, tmx, board, keypad));
    std::cout << "add keypad" << keypad->name << std::endl;
  }
  return sensors;
  // TODO: schedule periodic publishing
}

Mirte_Sensor::Mirte_Sensor(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::vector<uint8_t> pins, std::string name) {
  this->tmx = tmx;
  this->nh = nh;
  this->pins = pins;
  this->name = name;
  this->board = board;
}

KeypadMonitor::KeypadMonitor(std::shared_ptr<rclcpp::Node> nh,
                             std::shared_ptr<TMX> tmx,
                             std::shared_ptr<Mirte_Board> board,
                             std::shared_ptr<Keypad_data> keypad_data)
    : Mirte_Sensor(nh, tmx, board, {keypad_data->pin}, keypad_data->name) {
  this->keypad_data = keypad_data;
  keypad_pub = nh->create_publisher<mirte_msgs::msg::Keypad>(
      "/mirte/keypad/" + keypad_data->name, 1);
  keypad_pressed_pub = nh->create_publisher<mirte_msgs::msg::Keypad>(
      "/mirte/keypad/" + keypad_data->name + "_pressed", 1);
  keypad_service = nh->create_service<mirte_msgs::srv::GetKeypad>(
      keypad_data->name + "_get",
      std::bind(&KeypadMonitor::keypad_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  tmx->setPinMode(keypad_data->pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->add_analog_callback(keypad_data->pin, [this](auto pin, auto value) {
    this->callback(value);
  });
}

void KeypadMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}

bool KeypadMonitor::keypad_callback(
    const std::shared_ptr<mirte_msgs::srv::GetKeypad::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetKeypad::Response> res) {
  res->data = this->last_debounced_key;
  return true;
}

void KeypadMonitor::publish() {
  mirte_msgs::msg::Keypad msg;

  std::string key = "";
  auto maxValue = std::pow(2, board->get_adc_bits());
  auto scale = 4096 / maxValue;
  if (this->value < 70 / scale) {
    key = "left";
  } else if (this->value < 230 / scale) {
    key = "up";
  } else if (this->value < 410 / scale) {
    key = "down";
  } else if (this->value < 620 / scale) {
    key = "right";
  } else if (this->value < 880 / scale) {

    key = "enter";
  }
  // # Do some debouncing
  if (this->last_key != key) {
    this->last_debounce_time = nh->now().seconds();
  }

  std::string debounced_key = "";
  if (nh->now().seconds() - this->last_debounce_time > 0.1) {
    debounced_key = key;
  }

  // # Publish the last debounced key
  mirte_msgs::msg::Keypad keypad;
  keypad.key = debounced_key;
  this->keypad_pub->publish(keypad);

  // # check if we need to send a pressed message
  if (this->last_debounced_key != "" &&
      this->last_debounced_key != debounced_key) // TODO: check this
  {
    mirte_msgs::msg::Keypad pressed;
    // pressed.header = this->get_header();
    pressed.key = this->last_debounced_key;
    this->keypad_pressed_pub->publish(pressed);
  }
  this->last_key = key;
  this->last_debounced_key = debounced_key;
}

std::vector<std::shared_ptr<SonarMonitor>> SonarMonitor::get_sonar_monitors(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<SonarMonitor>> sensors;
  auto sonars = Sonar_data::parse_sonar_data(parser, board);
  for (auto sonar : sonars) {
    sensors.push_back(std::make_shared<SonarMonitor>(nh, tmx, board, sonar));
    std::cout << "add sonar" << sonar->name << std::endl;
  }
  return sensors;
}

SonarMonitor::SonarMonitor(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::shared_ptr<Sonar_data> sonar_data)
    : Mirte_Sensor(nh, tmx, board, {sonar_data->trigger, sonar_data->echo},
                   sonar_data->name) {
  this->sonar_data = sonar_data;
  sonar_pub = nh->create_publisher<sensor_msgs::msg::Range>(
      "/mirte/distance/" + sonar_data->name, 1);
  this->sonar_service = nh->create_service<mirte_msgs::srv::GetDistance>(
      "/mirte/get_distance_" + sonar_data->name,
      std::bind(&SonarMonitor::service_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  tmx->attach_sonar(sonar_data->trigger, sonar_data->echo,
                    [this](auto pin, auto value) { this->callback(value); });
}

void SonarMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}
void SonarMonitor::publish() {
  sensor_msgs::msg::Range msg;
  msg.header = this->get_header();
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = M_PI * 0.5; // 90 degrees, TODO: check real value
  msg.min_range = 0.02;
  msg.max_range = 1.5;
  msg.range = this->value / 1000.0;
  this->sonar_pub->publish(msg);
}

bool SonarMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetDistance::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetDistance::Response> res) {
  res->data = this->value / 1000.0;
  return true;
}

std::vector<std::shared_ptr<IntensityMonitor>>
IntensityMonitor::get_intensity_monitors(std::shared_ptr<rclcpp::Node> nh,
                                         std::shared_ptr<TMX> tmx,
                                         std::shared_ptr<Mirte_Board> board,
                                         std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;
  auto irs = Intensity_data::parse_intensity_data(parser, board);
  for (auto ir : irs) {
    if (ir->a_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<Analog_IntensityMonitor>(nh, tmx, board, ir));
      std::cout << "add analog intensity" << ir->name << std::endl;
    }
    if (ir->d_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<Digital_IntensityMonitor>(nh, tmx, board, ir));
      std::cout << "add digital intensity" << ir->name << std::endl;
    }
  }
  return sensors;
}

void Digital_IntensityMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}

void Digital_IntensityMonitor::publish() {
  mirte_msgs::msg::IntensityDigital msg;
  msg.value = this->value;
  this->intensity_pub->publish(msg);
}

Digital_IntensityMonitor::Digital_IntensityMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Intensity_data> intensity_data)
    : IntensityMonitor(nh, tmx, board, {intensity_data->d_pin},
                       intensity_data->name) {
  this->intensity_data = intensity_data;
  intensity_pub = nh->create_publisher<mirte_msgs::msg::IntensityDigital>(
      "/mirte/intensity/" + intensity_data->name + "_digital", 1);

  this->intensity_service =
      nh->create_service<mirte_msgs::srv::GetIntensityDigital>(
          "/mirte/get_intensity_" + intensity_data->name + "_digital",
          std::bind(&Digital_IntensityMonitor::service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
  tmx->setPinMode(intensity_data->d_pin, TMX::PIN_MODES::DIGITAL_INPUT, true,
                  0);
  tmx->add_digital_callback(
      intensity_data->d_pin,
      [this](auto pin, auto value) { this->callback(value); });
}

Analog_IntensityMonitor::Analog_IntensityMonitor(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board,
    std::shared_ptr<Intensity_data> intensity_data)
    : IntensityMonitor(nh, tmx, board, {intensity_data->a_pin},
                       intensity_data->name) {
  this->intensity_data = intensity_data;
  intensity_pub = nh->create_publisher<mirte_msgs::msg::Intensity>(
      "/mirte/intensity/" + intensity_data->name, 1);

  this->intensity_service = nh->create_service<mirte_msgs::srv::GetIntensity>(
      "/mirte/get_intensity_" + intensity_data->name,
      std::bind(&Analog_IntensityMonitor::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  tmx->setPinMode(intensity_data->a_pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->add_analog_callback(intensity_data->a_pin, [this](auto pin, auto value) {
    this->callback(value);
  });
}

void Analog_IntensityMonitor::callback(uint16_t value) {
  this->value = value;
  this->publish();
}

void Analog_IntensityMonitor::publish() {
  mirte_msgs::msg::Intensity msg;
  msg.value = this->value;
  this->intensity_pub->publish(msg);
}

bool Digital_IntensityMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetIntensityDigital::Response> res) {
  res->data = this->value;
  return true;
}

bool Analog_IntensityMonitor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::GetIntensity::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetIntensity::Response> res) {
  res->data = this->value;
  return true;
}
