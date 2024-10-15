#include <functional>
#include <memory>
// #include <tmx_cpp/modules/PCA9685.hpp>

// #include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/modules/pca/pca_motor.hpp>

using namespace std::placeholders;

// FIXME: Add to PCA module callback group
PCA_Motor::PCA_Motor(
  NodeData node_data, std::shared_ptr<PCA_Motor_data> motor_data,
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685)
{
  auto nh = node_data.nh;

  this->motor_data = motor_data;
  this->pca9685_mod = pca9685;

  RCLCPP_INFO(nh->get_logger(), "Added PCA Motor %s", motor_data->name.c_str());

  motor_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
    "motor/" + this->motor_data->name + "/set_speed",
    std::bind(&PCA_Motor::set_speed_service_callback, this, _1, _2));

  ros_client = nh->create_subscription<std_msgs::msg::Int32>(
    "motor/" + this->motor_data->name + "/speed", 1000,
    std::bind(&PCA_Motor::motor_callback, this, _1));
}

// TODO: This needs to be varified
// TODO: the speed is given as percentages.
std::tuple<uint32_t, uint32_t> PCA_Motor::calc_pwm_speed(int speed)
{
  int32_t speed_ = (int32_t)((float)speed * (4095.0) / 100.0);

  if (motor_data->invert) speed_ = -speed_;

  auto speedA = speed > 0 ? speed_ : 0;
  auto speedB = speed < 0 ? -speed_ : 0;

  return std::make_tuple(speedA, speedB);
}

void PCA_Motor::set_speed(int speed)
{
  if (this->last_speed == speed) return;

  bool reverse = false;
  // TODO: Could also do (last_speed*speed) < 0
  if ((last_speed < 0 && speed > 0) or (last_speed > 0 && speed < 0)) reverse = true;

  if (reverse) {
    pca9685_mod->set_pwm(motor_data->pinA, 0);
    pca9685_mod->set_pwm(motor_data->pinB, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto [speedA, speedB] = calc_pwm_speed(speed);

  pca9685_mod->set_pwm(motor_data->pinA, speedA);
  pca9685_mod->set_pwm(motor_data->pinB, speedB);

  last_speed = speed;
}

// // TODO: This function might do too much...
// void PCA_Motor::set_speed_old(
//   int speed, bool direct, std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals)
// {
//   std::cout << "Setting speed: " << speed << std::endl;
//   int32_t speed_ = (int32_t)((float)speed * (4095.0) / 100.0);
//   // TODO: add invert
//   if (this->motor_data->invert) {
//     speed_ = -speed;
//   }
//   //  if self.prev_motor_speed != speed:
//   //             change_dir = sign(self.prev_motor_speed) != sign(speed)
//
//   if (this->last_speed == speed) {
//     return;
//   }
//   bool reverse = false;
//   if (this->last_speed < 0 && speed > 0) {
//     reverse = true;
//   } else if (this->last_speed > 0 && speed < 0) {
//     reverse = true;
//   }
//
//   if (reverse && direct) {
//     this->pca9685_mod->set_pwm(this->motor_data->pinA, 0);
//     this->pca9685_mod->set_pwm(this->motor_data->pinB, 0);
//     // std::cout << "Setting speed to 0" << std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }
//   auto speedA = speed > 0 ? speed_ : 0;
//   auto speedB = speed < 0 ? -speed_ : 0;
//   if (direct) {
//     this->pca9685_mod->set_pwm(this->motor_data->pinA, speedA);
//     this->pca9685_mod->set_pwm(this->motor_data->pinB, speedB);
//   } else {
//     pwm_vals->push_back({this->motor_data->pinA, (uint16_t)speedA});
//     pwm_vals->push_back({this->motor_data->pinB, (uint16_t)speedB});
//   }
//   // std::cout << "Setting speed to " << std::dec << speed << std::endl;
//
//   // std::cout << "PCA Setting speed to " << std::dec << speed << std::endl;
//   this->last_speed = speed;
// }

std::vector<tmx_cpp::PCA9685_module::PWM_val> PCA_Motor::get_multi_speed_pwm(int speed)
{
  if (last_speed == speed) return {};

  auto [speedA, speedB] = calc_pwm_speed(speed);

  auto pwmA = tmx_cpp::PCA9685_module::PWM_val{this->motor_data->pinA, (uint16_t)speedA};
  auto pwmB = tmx_cpp::PCA9685_module::PWM_val{this->motor_data->pinB, (uint16_t)speedB};

  last_speed = speed;
  return {pwmA, pwmB};
}

void PCA_Motor::motor_callback(const std_msgs::msg::Int32 & msg) { this->set_speed(msg.data); }

void PCA_Motor::set_speed_service_callback(
  const std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetMotorSpeed::Response> res)
{
  this->set_speed(req->speed);
  res->status = true;
}
