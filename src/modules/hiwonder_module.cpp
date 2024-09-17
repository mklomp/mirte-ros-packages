#include <functional>

#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

// hiwonder bus
Hiwonder_bus_module::Hiwonder_bus_module(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::string name,
    std::shared_ptr<tmx_cpp::Modules> modules,
    std::shared_ptr<Hiwonder_bus_data> bus_data)
    : Mirte_module(nh, tmx, board, name), bus_data(bus_data) {

  std::function<void(std::vector<tmx_cpp::HiwonderServo_module::Servo_pos>)>
      position_cb = std::bind(&Hiwonder_bus_module::position_cb, this, _1);
  std::function<void(int, bool)> verify_cb =
      std::bind(&Hiwonder_bus_module::verify_cb, this, _1, _2);
  std::function<void(int, uint16_t, uint16_t)> range_cb =
      std::bind(&Hiwonder_bus_module::range_cb, this, _1, _2, _3);
  std::function<void(int, uint16_t)> offset_cb =
      std::bind(&Hiwonder_bus_module::offset_cb, this, _1, _2);

  std::vector<uint8_t> servo_ids;
  for (auto servo : bus_data->servos) {
    servo_ids.push_back(servo->id);
  }
  this->bus = std::make_shared<tmx_cpp::HiwonderServo_module>(
      this->bus_data->uart_port, this->bus_data->rx_pin, this->bus_data->tx_pin,
      servo_ids, position_cb, verify_cb, range_cb, offset_cb);
  modules->add_mod(this->bus);
  for (auto servo : bus_data->servos) {
    this->servos.push_back(
        std::make_shared<Hiwonder_servo>(nh, tmx, board, servo, this->bus));
  }

  // create bus services
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
      "set_" + this->name + "_all_servos_enable",
      std::bind(&Hiwonder_bus_module::enable_cb, this, _1, _2));
}

bool Hiwonder_bus_module::enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  this->bus->set_enabled_all(req->data);
  res->success = true;
  res->message = req->data ? "Enabled" : "Disabled";
  return true;
}

std::vector<std::shared_ptr<Hiwonder_bus_module>>
Hiwonder_bus_module::get_hiwonder_modules(std::shared_ptr<rclcpp::Node> nh,
                                          std::shared_ptr<tmx_cpp::TMX> tmx,
                                          std::shared_ptr<Mirte_Board> board,
                                          std::shared_ptr<Parser> parser,
                                          std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<Hiwonder_bus_module>> hiwonder_modules;
  auto hiwonder_data =
      Hiwonder_bus_data::parse_hiwonder_bus_data(parser, board);
  for (auto hiwonder : hiwonder_data) {
    auto hiwonder_module = std::make_shared<Hiwonder_bus_module>(
        nh, tmx, board, hiwonder->name, modules, hiwonder);
    hiwonder_modules.push_back(hiwonder_module);
  }
  return hiwonder_modules;
}

void Hiwonder_bus_module::position_cb(
    std::vector<tmx_cpp::HiwonderServo_module::Servo_pos> pos) {
  for (auto p : pos) {
    // std::cout << "Servo: " << (int)p.id << " pos: " << p.angle << std::endl;
    for (auto servo : this->servos) {
      if (servo->servo_data->id == p.id) {
        servo->position_cb(p);
        // servo->servo_data->angle = p.pos;
      }
    }
  }
}

// Following 3 callbacks are probably never used, maybe use verify to check that
// they exist
void Hiwonder_bus_module::verify_cb(int id, bool status) {
  std::cout << "Servo: " << id << " status: " << status << std::endl;
}

void Hiwonder_bus_module::range_cb(int id, uint16_t min, uint16_t max) {
  std::cout << "Servo: " << id << " range: " << min << " " << max << std::endl;
}

void Hiwonder_bus_module::offset_cb(int id, uint16_t offset) {
  std::cout << "Servo: " << id << " offset: " << offset << std::endl;
}

Hiwonder_servo::Hiwonder_servo(std::shared_ptr<rclcpp::Node> nh,
                               std::shared_ptr<tmx_cpp::TMX> tmx,
                               std::shared_ptr<Mirte_Board> board,
                               std::shared_ptr<Hiwonder_servo_data> servo_data,
                               std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod) {
  this->servo_data = servo_data;
  this->bus_mod = bus_mod;

  // create enable service
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
      "set_" + this->servo_data->name + "_servo_enable",
      std::bind(&Hiwonder_servo::enable_cb, this, _1, _2));

  // create angle service
  this->angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
      "set_" + this->servo_data->name + "_servo_angle",
      std::bind(&Hiwonder_servo::angle_cb, this, _1, _2));

  // create range service
  this->range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
      "get_" + this->servo_data->name + "_servo_range",
      std::bind(&Hiwonder_servo::range_cb, this, _1, _2));

  // create publisher
  this->position_pub = nh->create_publisher<mirte_msgs::msg::ServoPosition>(
      "servos/" + this->servo_data->name + "/position", 10);
}

void Hiwonder_servo::position_cb(tmx_cpp::HiwonderServo_module::Servo_pos &pos) {
  // TODO: publish current angle
  // don't forget to calculate angle
  auto angle = calc_angle_in(pos.angle);
  auto msg = mirte_msgs::msg::ServoPosition();
  msg.angle = angle;
  msg.raw = pos.angle;
  msg.header.frame_id = this->servo_data->frame_id;
  // msg.header.stamp = this->nh->now();
  this->position_pub->publish(msg);
}

bool Hiwonder_servo::enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  this->bus_mod->set_enable_servo(this->servo_data->id, req->data);
  res->success = true;
  res->message = req->data ? "Enabled" : "Disabled";
  return true;
}

bool Hiwonder_servo::angle_cb(
    const std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetServoAngle::Response> res) {
  auto angle = calc_angle_out(req->angle);
  this->bus_mod->set_single_servo(this->servo_data->id, angle, 100);
  res->status = true;
  return true;
}

bool Hiwonder_servo::range_cb(
    const std::shared_ptr<mirte_msgs::srv::GetServoRange::Request> req,
    std::shared_ptr<mirte_msgs::srv::GetServoRange::Response> res) {
  res->min = this->servo_data->min_angle_in;
  res->max = this->servo_data->max_angle_in;
  return true;
}

template <typename T> T scale(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Hiwonder_servo::calc_angle_out(float angle) {

  float angle_out = scale<float>(
      angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
      this->servo_data->min_angle_out, this->servo_data->max_angle_out);
  if (this->servo_data->invert) {
    angle_out = scale<float>(
        angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
        this->servo_data->max_angle_out, this->servo_data->min_angle_out);
  }
  return std::max(std::min((int)angle_out, this->servo_data->max_angle_out),
                  this->servo_data->min_angle_out);
}

float Hiwonder_servo::calc_angle_in(uint16_t angle) {
  float angle_in = scale<float>(
      angle, this->servo_data->min_angle_out, this->servo_data->max_angle_out,
      this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  if (this->servo_data->invert) {
    angle_in = scale<float>(
        angle, this->servo_data->max_angle_out, this->servo_data->min_angle_out,
        this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  }
  return angle_in;
}