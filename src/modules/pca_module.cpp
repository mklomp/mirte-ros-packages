#include <algorithm>
#include <functional>

#include <mirte_telemetrix_cpp/modules/pca_module.hpp>

using namespace std::placeholders;  // for _1, _2, _3...

std::vector<std::shared_ptr<PCA_Module>> PCA_Module::get_pca_modules(
  NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules)
{
  std::vector<std::shared_ptr<PCA_Module>> pca_modules;
  auto pca_data = parse_all_modules<PCAData>(parser, node_data.board);
  for (auto pca : pca_data) {
    auto pca_module = std::make_shared<PCA_Module>(node_data, pca, modules);
    pca_modules.push_back(pca_module);
  }
  return pca_modules;
}

// TODO: This possibly could be a reentrant (parallel) callback group
PCA_Module::PCA_Module(
  NodeData node_data, PCAData pca_data, std::shared_ptr<tmx_cpp::Modules> modules)
: Mirte_module(node_data, {pca_data.scl, pca_data.sda}, (ModuleData)pca_data)
{
  tmx->setI2CPins(pca_data.sda, pca_data.scl, pca_data.port);

  this->pca9685 =
    std::make_shared<tmx_cpp::PCA9685_module>(pca_data.port, pca_data.addr, pca_data.frequency);

  modules->add_mod(pca9685);
  for (auto motor : pca_data.motors) {
    this->motors.push_back(std::make_shared<PCA_Motor>(node_data, motor, pca9685));
  }
  // TODO: add servos to this as well

  motor_service = nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
    "motor/" + this->name + "/set_multiple_speeds",
    std::bind(&PCA_Module::set_multi_speed_service_callback, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

bool PCA_Module::set_multi_speed_service_callback(
  const std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetSpeedMultiple::Response> res)
{
  // TODO: Why is this a shared_ptr? (Probably unneccessary)
  std::shared_ptr<std::vector<tmx_cpp::PCA9685_module::PWM_val>> pwm_vals =
    std::make_shared<std::vector<tmx_cpp::PCA9685_module::PWM_val>>();
  if (req->speeds.size() == 0) {
    res->success = false;
    return false;
  }

  for (auto speed : req->speeds) {
    auto name = speed.name;
    auto motor = std::find_if(
      motors.begin(), motors.end(), [name](auto motor) { return motor->motor_data->name == name; });

    if (motor == motors.end()) {
      RCLCPP_ERROR(
        logger, "PCA Motor '%s' could not be found. Ignored for set multi speed command",
        name.c_str());
      continue;
    }

    auto motor_pwm_vals = (*motor)->get_multi_speed_pwm(speed.speed);
    pwm_vals->insert(pwm_vals->end(), motor_pwm_vals.begin(), motor_pwm_vals.end());
  }

  if (pwm_vals->size() > 0) pca9685->set_multiple_pwm(pwm_vals);

  res->success = true;
  return true;
}
