#include "mirte-sensors.hpp"
#include "parsers/sensors.hpp"
Mirte_Sensors::Mirte_Sensors(
  node_handle nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser)
{
  this->tmx = tmx;
  this->nh = nh;
  this->board = board;
  auto keypads = KeypadMonitor::get_keypad_monitors(nh, tmx, board, parser);
  this->sensors.insert(this->sensors.end(), keypads.begin(), keypads.end());
}

std::vector<std::shared_ptr<KeypadMonitor>>
KeypadMonitor::get_keypad_monitors(
  node_handle nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<KeypadMonitor>> sensors;
  auto keypads = Keypad_data::parse_keypad_data(parser, board);
  for (auto keypad : keypads) {
    sensors.push_back(std::make_shared<KeypadMonitor>(nh, tmx, board, keypad));
  }
  return sensors;
  // TODO: schedule periodic publishing
}

Mirte_Sensor::Mirte_Sensor(
  node_handle nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::vector<uint8_t> pins, std::string name)
{
  this->tmx = tmx;
  this->nh = nh;
  this->pins = pins;
  this->name = name;
  this->board = board;
}

KeypadMonitor::KeypadMonitor(
  node_handle nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Keypad_data> keypad_data)
: Mirte_Sensor(nh, tmx, board, {keypad_data->pin}, keypad_data->name)
{
  this->keypad_data = keypad_data;
  keypad_pub =
    nh->create_publisher<mirte_msgs::msg::Keypad>("/mirte/keypad/" + keypad_data->name, 1);
  keypad_pressed_pub =
    nh->create_publisher<mirte_msgs::msg::Keypad>(
    "/mirte/keypad/" + keypad_data->name + "_pressed",
    1);
  keypad_service =
    nh->create_service<mirte_msgs::srv::GetKeypad>(
    keypad_data->name + "_get",
    std::bind(&KeypadMonitor::keypad_callback, this, std::placeholders::_1, std::placeholders::_2));
  tmx->setPinMode(keypad_data->pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->add_analog_callback(
    keypad_data->pin, [this](auto pin, auto value) {this->callback(value);});
}

void KeypadMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}

bool KeypadMonitor::keypad_callback(
  const std::shared_ptr<mirte_msgs::srv::GetKeypad::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetKeypad::Response> res)
{
  res->data = this->last_debounced_key;
  return true;
}

void KeypadMonitor::publish()
{
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
    this->last_debounced_key != debounced_key)     // TODO: check this
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
  node_handle nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<SonarMonitor>> sensors;
  auto sonars = Sonar_data::parse_sonar_data(parser, board);
  for (auto sonar : sonars) {
    sensors.push_back(std::make_shared<SonarMonitor>(nh, tmx, board, sonar));
  }
  return sensors;
}


SonarMonitor::SonarMonitor(
  node_handle nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Sonar_data> sonar_data)
: Mirte_Sensor(
    nh, tmx, board, {sonar_data->trigger, sonar_data->echo}, sonar_data->name)
{
  this->sonar_data = sonar_data;
  sonar_pub = nh->create_publisher<sensor_msgs_range>("/mirte/distance/" + sonar_data->name, 1);
  this->sonar_service =
    nh->create_service<mirte_msgs_get_distance>(
    "/mirte/get_distance_" + sonar_data->name,
    std::bind(&SonarMonitor::service_callback, this, std::placeholders::_1, std::placeholders::_2));
  tmx->attach_sonar(
    sonar_data->trigger, sonar_data->echo, [this](auto pin, auto value) {
      this->callback(value);
    });
}

void SonarMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}
void SonarMonitor::publish()
{
  sensor_msgs_range msg;
  msg.header = this->get_header();
  msg.radiation_type = sensor_msgs_range::ULTRASOUND;
  msg.field_of_view = M_PI * 0.5; // 90 degrees, TODO: check real value
  msg.min_range = 0.02;
  msg.max_range = 1.5;
  msg.range = this->value / 1000.0;
  this->sonar_pub->publish(msg);
}

bool SonarMonitor::service_callback(
  const std::shared_ptr<mirte_msgs_get_distance::Request> req,
  std::shared_ptr<mirte_msgs_get_distance::Response> res)
{
  res->data = this->value / 1000.0;
  return true;
}

std::vector<std::shared_ptr<IntensityMonitor>> IntensityMonitor::get_intensity_monitors(
  node_handle nh, std::shared_ptr<TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Parser> parser)
{
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;
  auto irs = Intensity_data::parse_intensity_data(parser, board);
  for (auto ir : irs) {
    sensors.push_back(std::make_shared<IntensityMonitor>(nh, tmx, board, ir));
  }
  return sensors;
}

IntensityMonitor::IntensityMonitor(
  node_handle nh, std::shared_ptr<TMX> tmx,
  std::shared_ptr<Mirte_Board> board,
  std::shared_ptr<Intensity_data> intensity_data) : Mirte_Sensor(
    nh, tmx, board, {intensity_data->a_pin, intensity_data->d_pin}, intensity_data->name) {
  this->intensity_data = intensity_data;
  intensity_pub =
    nh->create_publisher<sensor_msgs_range>("/mirte/intensity/" + intensity_data->name, 1);
  tmx->setPinMode(intensity_data->a_pin, TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  tmx->setPinMode(intensity_data->d_pin, TMX::PIN_MODES::DIGITAL_INPUT, true, 0);
  tmx->add_analog_callback(
    intensity_data->a_pin, [this](auto pin, auto value) {this->callback(value);});
  tmx->add_digital_callback(
    intensity_data->d_pin, [this](auto pin, auto value) {this->analog_callback(value);});
}

void IntensityMonitor::callback(uint16_t value)
{
  this->value = value;
  this->publish();
}
