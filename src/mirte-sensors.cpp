#include "mirte_telemetrix_cpp/mirte-sensors.hpp"
#include "mirte_telemetrix_cpp/parsers/p_sensors.hpp"

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/keypad_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/sonar_monitor.hpp>

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

  auto encoders = EncoderMonitor::get_encoder_monitors(nh, tmx, board, parser);
  this->sensors.insert(this->sensors.end(), encoders.begin(), encoders.end());

  this->timer = nh->create_wall_timer(
      std::chrono::milliseconds(1000 / parser->get_frequency()),
      std::bind(&Mirte_Sensors::publish, this));
  this->pin_service = nh->create_service<mirte_msgs::srv::GetPinValue>(
      "get_pin_value",
      std::bind(&Mirte_Sensors::pin_callback, this, std::placeholders::_1,
                std::placeholders::_2));
}

bool Mirte_Sensors::pin_callback(
    const std::shared_ptr<mirte_msgs::srv::GetPinValue::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetPinValue::Response> res) {
  bool is_digital = starts_with(req->type, "d") || starts_with(req->type, "D");
  auto pin = this->board->resolvePin(req->pin);
  bool has_digital_cb = false;
  bool has_analog_cb = false;
  if (this->pin_map.count(pin)) {
    const auto [type, value, ana_cb, dig_cb] = this->pin_map[pin];
    has_analog_cb = ana_cb;
    has_digital_cb = dig_cb;
    if ((type == PIN_USE::DIGITAL_IN && is_digital) ||
        (type == PIN_USE::ANALOG_IN && !is_digital)) {
      res->data = value;
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
    const auto [type, value, has_analog, has_digital] = this->pin_map[pin];
    if (value != -1) {
      res->data = value;
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



