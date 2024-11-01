#include "mirte_telemetrix_cpp/mirte-sensors.hpp"

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/keypad_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/sonar_monitor.hpp>

Mirte_Sensors::Mirte_Sensors(NodeData node_data, std::shared_ptr<Parser> parser)
: tmx(node_data.tmx), nh(node_data.nh), board(node_data.board)
{
  using namespace std::placeholders;

  auto keypads = KeypadMonitor::get_keypad_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), keypads.begin(), keypads.end());

  auto sonars = SonarMonitor::get_sonar_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), sonars.begin(), sonars.end());

  auto irs = IntensityMonitor::get_intensity_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), irs.begin(), irs.end());

  auto encoders = EncoderMonitor::get_encoder_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), encoders.begin(), encoders.end());

  this->pin_service = nh->create_service<mirte_msgs::srv::GetPinValue>(
    "get_pin_value", std::bind(&Mirte_Sensors::pin_callback, this, _1, _2));
}

void Mirte_Sensors::pin_callback(
  const mirte_msgs::srv::GetPinValue::Request::ConstSharedPtr req,
  mirte_msgs::srv::GetPinValue::Response::SharedPtr res)
{
  bool is_digital = starts_with(req->type, "d") || starts_with(req->type, "D");
  auto pin = this->board->resolvePin(req->pin);
  bool has_digital_cb = false;
  bool has_analog_cb = false;
  if (this->pin_map.count(pin)) {
    const auto [type, value, ana_cb, dig_cb] = this->pin_map[pin];
    has_analog_cb = ana_cb;
    has_digital_cb = dig_cb;
    if (
      (type == PIN_USE::DIGITAL_IN && is_digital) || (type == PIN_USE::ANALOG_IN && !is_digital)) {
      res->data = value;
      return;
    }
  }
  if (is_digital) {
    this->pin_map[pin] = {PIN_USE::DIGITAL_IN, -1, has_analog_cb, true};

    this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT, true);
    if (!has_digital_cb) {
      std::cout << "add digital callback" << std::endl;
      this->tmx->add_digital_callback(pin, [this](auto pin, auto value) {
        const auto [type, old_value, has_analog, has_digital] = this->pin_map[pin];
        if (type == PIN_USE::DIGITAL_IN && old_value != value) {
          this->pin_map[pin] = {type, value, has_analog, has_digital};
        }
      });
    }
  } else {
    this->pin_map[pin] = {PIN_USE::ANALOG_IN, -1, true, has_digital_cb};

    this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::ANALOG_INPUT, true, 0);
    if (!has_analog_cb) {
      std::cout << "add a callback" << std::endl;

      this->tmx->add_analog_callback(pin, [this](auto pin, auto value) {
        const auto [type, old_value, has_analog, has_digital] = this->pin_map[pin];
        if (type == PIN_USE::ANALOG_IN && old_value != value) {
          this->pin_map[pin] = {type, value, has_analog, has_digital};
        }
      });
    }
  }

  // while time less than 5s, after that its probably not going to change
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < std::chrono::seconds(5)) {
    const auto [type, value, has_analog, has_digital] = this->pin_map[pin];
    if (value != -1) {
      res->data = value;
      return;
    }

    // sleep for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  res->data = -1;
}

void Mirte_Sensors::stop()
{
  for (auto sensor : this->sensors) {
    sensor->stop();
  }
}
