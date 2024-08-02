#include <functional>
#include <mirte-modules.hpp>
  using namespace std::placeholders; // for _1, _2, _3...

Mirte_modules::Mirte_modules(std::shared_ptr<rclcpp::Node> nh,
                             std::shared_ptr<TMX> tmx,
                             std::shared_ptr<Mirte_Board> board,
                             std::shared_ptr<Parser> parser) {
  this->nh = nh;
  this->tmx = tmx;
  this->board = board;
  // this->parser = parser;
  this->module_sys = std::make_shared<Modules>(tmx);
  auto pca_mods =
      PCA_Module::get_pca_modules(nh, tmx, board, parser, this->module_sys);
  this->modules.insert(this->modules.end(), pca_mods.begin(), pca_mods.end());

  auto hiwonder_mods = Hiwonder_bus_module::get_hiwonder_modules(
      nh, tmx, board, parser, this->module_sys);
  std::cout << "Adding hiwonder modules" << hiwonder_mods.size() <<  std::endl;
  this->modules.insert(this->modules.end(), hiwonder_mods.begin(),
                       hiwonder_mods.end());
}
std::vector<std::shared_ptr<PCA_Module>> PCA_Module::get_pca_modules(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser,
    std::shared_ptr<Modules> modules) {
  std::vector<std::shared_ptr<PCA_Module>> pca_modules;
  auto pca_data = PCA_data::parse_pca_data(parser, board);
  for (auto pca : pca_data) {
    auto pca_module =
        std::make_shared<PCA_Module>(nh, tmx, board, pca->name, modules, pca);
    pca_modules.push_back(pca_module);
  }
  return pca_modules;
}
Mirte_module::Mirte_module(std::shared_ptr<rclcpp::Node> nh,
                           std::shared_ptr<TMX> tmx,
                           std::shared_ptr<Mirte_Board> board,
                           std::string name) {
  this->name = name;
  this->tmx = tmx;
  this->nh = nh;
  this->board = board;
}

PCA_Module::PCA_Module(std::shared_ptr<rclcpp::Node> nh,
                       std::shared_ptr<TMX> tmx,
                       std::shared_ptr<Mirte_Board> board, std::string name,
                       std::shared_ptr<Modules> modules,
                       std::shared_ptr<PCA_data> pca_data)
    : Mirte_module(nh, tmx, board, name) {
      std::cout << "set i2c" << std::dec << (int)pca_data->scl<< "sda"<< std::dec << (int)pca_data->sda<<  "sda"<<std::dec << (int)pca_data->port << std::endl;
  tmx->setI2CPins(pca_data->scl, pca_data->sda, pca_data->port);
  this->pca9685 = std::make_shared<PCA9685_module>(
      pca_data->port, pca_data->addr, pca_data->frequency);
  modules->add_mod(pca9685);
  for (auto motor : pca_data->motors) {
    std::cout << "Adding pcaaaa motor: " << motor->name << std::endl;
    this->motors.push_back(
        std::make_shared<PCA_Motor>(nh, tmx, board, motor, pca9685));
  }
  // TODO: add servos to this as well
}

PCA_Motor::PCA_Motor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                     std::shared_ptr<Mirte_Board> board,
                     std::shared_ptr<PCA_Motor_data> motor_data,
                     std::shared_ptr<PCA9685_module> pca9685) {
  this->motor_data = motor_data;
  this->pca9685_mod = pca9685;
  // this->tmx = tmx;
  // this->nh = nh;
  // this->board = board;
  motor_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
      "/mirte/set_" + this->motor_data->name + "_speed",
      std::bind(&PCA_Motor::service_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  ros_client = nh->create_subscription<std_msgs::msg::Int32>(
      "/mirte/motor_" + this->motor_data->name + "_speed", 1000,
      std::bind(&PCA_Motor::motor_callback, this, std::placeholders::_1));
}

void PCA_Motor::set_speed(int speed) {
  std::cout << "Setting speed: " << speed << std::endl;
  int32_t speed_ = (int32_t)((float)speed * (4095.0) / 100.0);
  // TODO: add invert
  if (this->motor_data->invert) {
    speed_ = -speed;
  }
  //  if self.prev_motor_speed != speed:
  //             change_dir = sign(self.prev_motor_speed) != sign(speed)

  if (this->last_speed == speed) {
    return;
  }
  bool reverse = false;
  if (this->last_speed < 0 && speed > 0) {
    reverse = true;
  } else if (this->last_speed > 0 && speed < 0) {
    reverse = true;
  }

  if (reverse) {
    this->pca9685_mod->set_pwm(this->motor_data->pinA, 0);
    this->pca9685_mod->set_pwm(this->motor_data->pinB, 0);
    std::cout << "Setting speed to 0" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  this->pca9685_mod->set_pwm(this->motor_data->pinA, speed > 0 ? speed_ : 0);
  this->pca9685_mod->set_pwm(this->motor_data->pinB, speed < 0 ? -speed_ : 0);
  std::cout << "Setting speed to " << std::dec << speed << std::endl;

  std::cout << "PCA Setting speed to " << std::dec << speed << std::endl;
  this->last_speed = speed;
  //   pca9685->setPWM(0, speed);
}
void PCA_Motor::motor_callback(const std_msgs::msg::Int32 &msg) {
  this->set_speed(msg.data);
}
bool PCA_Motor::service_callback(
    const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res) {
  this->set_speed(req->speed);
  res->status = true;
  return true;
}

// hiwonder bus
Hiwonder_bus_module::Hiwonder_bus_module(
    std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
    std::shared_ptr<Mirte_Board> board, std::string name,
    std::shared_ptr<Modules> modules,
    std::shared_ptr<Hiwonder_bus_data> bus_data)
    : Mirte_module(nh, tmx, board, name) {

  std::function<void(std::vector<HiwonderServo_module::Servo_pos>)>
      position_cb = std::bind(&Hiwonder_bus_module::position_cb, this, _1);
  std::function<void(int, bool)> verify_cb =
      std::bind(&Hiwonder_bus_module::verify_cb, this, _1, _2);
  std::function<void(int, uint16_t, uint16_t)> range_cb =
      std::bind(&Hiwonder_bus_module::range_cb, this, _1, _2, _3);
  std::function<void(int, uint16_t)> offset_cb =
      std::bind(&Hiwonder_bus_module::offset_cb, this, _1, _2);
  this->bus_data = bus_data;
  std::vector<uint8_t> servo_ids;
  for (auto servo : bus_data->servos) {
    servo_ids.push_back(servo->id);
  }
  this->bus = std::make_shared<HiwonderServo_module>(
      this->bus_data->uart_port, this->bus_data->rx_pin, this->bus_data->tx_pin,
      servo_ids, position_cb, verify_cb, range_cb, offset_cb);
  modules->add_mod(this->bus);
  for (auto servo : bus_data->servos) {
    this->servos.push_back(
        std::make_shared<Hiwonder_servo>(nh, tmx, board, servo, this->bus));
  }

  // create bus services
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
      "/mirte/set_" + this->name + "_all_servos_enable",
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
                                          std::shared_ptr<TMX> tmx,
                                          std::shared_ptr<Mirte_Board> board,
                                          std::shared_ptr<Parser> parser,
                                          std::shared_ptr<Modules> modules) {
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

void Hiwonder_bus_module::position_cb(std::vector<HiwonderServo_module::Servo_pos> pos) {
  for (auto p : pos) {
    // std::cout << "Servo: " << (int)p.id << " pos: " << p.angle << std::endl;
    for(auto servo : this->servos) {
      if (servo->servo_data->id == p.id) {
        servo->position_cb(p);
        // servo->servo_data->angle = p.pos;
      }
    }
  }
}

// Following 3 callbacks are probably never used, maybe use verify to check that they exist
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
                               std::shared_ptr<TMX> tmx,
                               std::shared_ptr<Mirte_Board> board,
                               std::shared_ptr<Hiwonder_servo_data> servo_data,
                               std::shared_ptr<HiwonderServo_module> bus_mod) {
  this->servo_data = servo_data;
  this->bus_mod = bus_mod;
  // this->tmx = tmx;
  // this->nh = nh;
  // this->board = board;
  
  // create enable service
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
      "/mirte/set_" + this->servo_data->name + "_servo_enable",
      std::bind(&Hiwonder_servo::enable_cb, this, _1, _2));
  
  // create angle service
  this->angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
      "/mirte/set_" + this->servo_data->name + "_servo_angle",
      std::bind(&Hiwonder_servo::angle_cb, this, _1,_2));

  // create range service
  this->range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
      "/mirte/get_" + this->servo_data->name + "_servo_range",
      std::bind(&Hiwonder_servo::range_cb, this, _1, _2));

  // create publisher
  this->position_pub = nh->create_publisher<mirte_msgs::msg::ServoPosition>(
      "/servos/" + this->servo_data->name + "/position", 10);
  
}


void Hiwonder_servo::position_cb(HiwonderServo_module::Servo_pos& pos) {
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

template <typename T>
T scale(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Hiwonder_servo::calc_angle_out(float angle) {

  float angle_out = scale<float>(angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in, this->servo_data->min_angle_out, this->servo_data->max_angle_out);
  if(this->servo_data->invert) {
  angle_out = scale<float>(angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in, this->servo_data->max_angle_out, this->servo_data->min_angle_out);
  }
  return std::max(std::min((int)angle_out, this->servo_data->max_angle_out), this->servo_data->min_angle_out);
}

float Hiwonder_servo::calc_angle_in(uint16_t angle) {
  float angle_in = scale<float>(angle, this->servo_data->min_angle_out, this->servo_data->max_angle_out, this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  if(this->servo_data->invert) {
  angle_in = scale<float>(angle, this->servo_data->max_angle_out, this->servo_data->min_angle_out, this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  }
  return angle_in;
}