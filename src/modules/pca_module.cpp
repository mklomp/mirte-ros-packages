#include <functional>

#include <mirte_telemetrix_cpp/modules/pca_module.hpp>

using namespace std::placeholders;  // for _1, _2, _3...

std::vector<std::shared_ptr<PCA_Module>> PCA_Module::get_pca_modules(
  NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules)
{
  std::vector<std::shared_ptr<PCA_Module>> pca_modules;
  auto pca_data = PCA_data::parse_pca_data(parser, node_data.board);
  for (auto pca : pca_data) {
    auto pca_module = std::make_shared<PCA_Module>(node_data, pca->name, modules, pca);
    pca_modules.push_back(pca_module);
  }
  return pca_modules;
}

PCA_Module::PCA_Module(
  NodeData node_data, std::string name, std::shared_ptr<tmx_cpp::Modules> modules,
  std::shared_ptr<PCA_data> pca_data)
: Mirte_module(node_data, name)
{
  tmx->setI2CPins(pca_data->scl, pca_data->sda, pca_data->port);

  this->pca9685 =
    std::make_shared<tmx_cpp::PCA9685_module>(pca_data->port, pca_data->addr, pca_data->frequency);

  modules->add_mod(pca9685);
  for (auto motor : pca_data->motors) {
    std::cout << "Adding PCA motor: " << motor->name << std::endl;
    this->motors.push_back(std::make_shared<PCA_Motor>(node_data, motor, pca9685));
  }
  // TODO: add servos to this as well

  motor_service = nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
    "set_" + this->name + "_multiple_speeds",
    std::bind(&PCA_Module::motor_service_cb, this, _1, _2));
}

bool PCA_Module::motor_service_cb(
  const std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Response> res)
{
  std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals =
    std::make_shared<std::vector<tmx_cpp::PCA9685_module::PWM_val>>();
  if (req->speeds.size() == 0) {
    res->success = false;
    return false;
  }
  for (auto speed : req->speeds) {
    for (auto motor : this->motors) {
      if (motor->motor_data->name == speed.name) {
        motor->set_speed(speed.speed, false, pwm_vals);
        continue;
      }
    }
  }
  if (pwm_vals->size() > 0) {
    this->pca9685->set_multiple_pwm(pwm_vals);
  }

  res->success = true;
  return true;
}

PCA_Motor::PCA_Motor(
  NodeData node_data, std::shared_ptr<PCA_Motor_data> motor_data,
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685)
{
  auto nh = node_data.nh;

  this->motor_data = motor_data;
  this->pca9685_mod = pca9685;
  // this->tmx = tmx;
  // this->nh = nh;
  // this->board = board;
  motor_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
    "set_" + this->motor_data->name + "_speed",
    std::bind(&PCA_Motor::service_callback, this, std::placeholders::_1, std::placeholders::_2));

  ros_client = nh->create_subscription<std_msgs::msg::Int32>(
    "motor_" + this->motor_data->name + "_speed", 1000,
    std::bind(&PCA_Motor::motor_callback, this, std::placeholders::_1));
}

void PCA_Motor::set_speed(
  int speed, bool direct, std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals)
{
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

  if (reverse && direct) {
    this->pca9685_mod->set_pwm(this->motor_data->pinA, 0);
    this->pca9685_mod->set_pwm(this->motor_data->pinB, 0);
    // std::cout << "Setting speed to 0" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto speedA = speed > 0 ? speed_ : 0;
  auto speedB = speed < 0 ? -speed_ : 0;
  if (direct) {
    this->pca9685_mod->set_pwm(this->motor_data->pinA, speedA);
    this->pca9685_mod->set_pwm(this->motor_data->pinB, speedB);
  } else {
    pwm_vals->push_back({this->motor_data->pinA, (uint16_t)speedA});
    pwm_vals->push_back({this->motor_data->pinB, (uint16_t)speedB});
  }
  // std::cout << "Setting speed to " << std::dec << speed << std::endl;

  // std::cout << "PCA Setting speed to " << std::dec << speed << std::endl;
  this->last_speed = speed;
}
void PCA_Motor::motor_callback(const std_msgs::msg::Int32 & msg) { this->set_speed(msg.data); }
bool PCA_Motor::service_callback(
  const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res)
{
  this->set_speed(req->speed);
  res->status = true;
  return true;
}