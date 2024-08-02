#include <mirte-modules.hpp>

Mirte_modules::Mirte_modules(std::shared_ptr<rclcpp::Node> nh,
                             std::shared_ptr<TMX> tmx,
                             std::shared_ptr<Mirte_Board> board,
                             std::shared_ptr<Parser> parser) {
  this->nh = nh;
  this->tmx = tmx;
  this->board = board;
  // this->parser = parser;
  this->module_sys = std::make_shared<Modules>(tmx);
  auto pca_mods = PCA_Module::get_pca_modules(nh, tmx,board, parser, this->module_sys);
        this->modules.insert(this->modules.end(), pca_mods.begin(), pca_mods.end());
}
std::vector<std::shared_ptr<PCA_Module>> PCA_Module::get_pca_modules(
      std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
      std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser, std::shared_ptr<Modules> modules){
        std::vector<std::shared_ptr<PCA_Module>> pca_modules;
        auto pca_data = PCA_data::parse_pca_data(parser, board);
        for(auto pca: pca_data) {
                auto pca_module = std::make_shared<PCA_Module>(nh, tmx, board, pca->name, modules,  pca);
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
                       std::shared_ptr<Mirte_Board> board,
                       std::string name,
                     std::shared_ptr<Modules> modules, std::shared_ptr<PCA_data> pca_data)
    : Mirte_module(nh, tmx, board, name) {
      tmx->setI2CPins(pca_data->scl, pca_data->sda, pca_data->port);
  this->pca9685 = std::make_shared<PCA9685_module>(pca_data->port, pca_data->addr, pca_data->frequency);
  modules->add_mod(pca9685);
  for(auto motor: pca_data->motors) {
    std::cout << "Adding pcaaaa motor: " << motor->name << std::endl;
                        this->motors.push_back(std::make_shared<PCA_Motor>(nh, tmx, board, motor, pca9685));
                }
                // TODO: add servos to this as well
}


PCA_Motor::PCA_Motor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<TMX> tmx,
                     std::shared_ptr<Mirte_Board> board, std::shared_ptr<PCA_Motor_data> motor_data, std::shared_ptr<PCA9685_module> pca9685) {
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
  if(this->motor_data->invert) {
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



