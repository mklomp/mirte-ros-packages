#include <functional>

#include <mirte_telemetrix_cpp/modules/ina226_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

INA226_sensor::INA226_sensor(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::string name, std::shared_ptr<tmx_cpp::Sensors> modules,
  std::shared_ptr<INA226_data> ina_data)
: Mirte_module(nh, tmx, board, name)
{
  tmx->setI2CPins(ina_data->scl, ina_data->sda, ina_data->port);

  this->ina_data = ina_data;
  this->used_time = nh->now();
  this->total_used_mAh = 0;

  this->ina226 = std::make_shared<tmx_cpp::INA226_module>(
    ina_data->port, ina_data->addr, std::bind(&INA226_sensor::data_cb, this, _1, _2));
  this->battery_pub =
    nh->create_publisher<sensor_msgs::msg::BatteryState>("power/" + this->ina_data->name, 1);

  this->used_pub =
    nh->create_publisher<std_msgs::msg::Int32>("power/" + this->ina_data->name + "/used", 1);

  this->shutdown_service = nh->create_service<std_srvs::srv::SetBool>(
    "shutdown", std::bind(&INA226_sensor::shutdown_robot_cb, this, _1, _2));

  modules->add_sens(this->ina226);
  // TODO: add used topic
  // TODO: add shutdown service
  // TODO: add auto shutdown
}

void INA226_sensor::data_cb(float voltage, float current)
{
  // std::cout << "INA226 data: " << current << " " << voltage << std::endl;
  auto msg = sensor_msgs::msg::BatteryState();
  msg.voltage = voltage;
  msg.current = current;
  msg.header.frame_id = this->ina_data->name;
  msg.header.stamp = this->nh->now();
  msg.percentage = calc_soc(voltage);
  this->battery_pub->publish(msg);
  this->integrate_usage(current);
  this->check_soc(voltage, current);
}

float INA226_sensor::calc_soc(float voltage)
{
  const auto CELL_COUNT = 3;
  voltage = voltage / CELL_COUNT;
  std::vector<std::pair<float, float>> soc_levels{
    // single cell voltages vs percentage
    {0, 0},       {3.27, 0.00}, {3.61, 0.05}, {3.69, 0.10}, {3.71, 0.15}, {3.73, 0.20},
    {3.75, 0.25}, {3.77, 0.30}, {3.79, 0.35}, {3.80, 0.40}, {3.82, 0.45}, {3.84, 0.50},
    {3.85, 0.55}, {3.87, 0.60}, {3.91, 0.65}, {3.95, 0.70}, {3.98, 0.75}, {4.02, 0.80},
    {4.08, 0.85}, {4.11, 0.90}, {4.15, 0.95}, {4.20, 1.00}, {10, 1.00}};
  for (size_t i = 1; i < soc_levels.size(); i++) {
    if (voltage < soc_levels[i].first) {  // find the first voltage level that is
                                          // higher than the current voltage
      return soc_levels[i - 1].second;    // return the soc level of the previous voltage level
    }
  }
  return 100;
}

void INA226_sensor::integrate_usage(float current)
{
  auto current_time = this->nh->now();
  auto duration = current_time - this->used_time;

  auto used_mA_sec = duration.seconds() * current * 1000.0;  // milliAmpere seconds
  auto used_mAh = used_mA_sec / 3600;                        // convert to mAh

  this->total_used_mAh += used_mAh;
  this->used_time = current_time;
  std_msgs::msg::Int32 msg;
  msg.data = (int32_t)this->total_used_mAh;
  this->used_pub->publish(msg);
}

void INA226_sensor::check_soc(float voltage, float current)
{
  if (!this->enable_turn_off) {
    if (voltage > 6 && current > 0.1) {
      this->enable_turn_off = true;
      this->in_power_dip = false;
      std::cout << "Enabling turn off" << std::endl;
    }
    return;
  }
  //  at start dip of too low voltage, start timer, when longer than 5s below
  //  trigger voltage, then shut down
  // this makes sure that a short dip (motor start) does not trigger it

  if (voltage != 0.1 && voltage < this->ina_data->min_voltage) {
    if (!this->in_power_dip) {
      this->turn_off_trigger_time = this->nh->now();
      this->in_power_dip = true;
      std::cout << "Triggering turn off in " << this->ina_data->power_low_time << "s" << std::endl;
    }
  } else {
    this->in_power_dip = false;
    this->turn_off_trigger_time = rclcpp::Time(0, 0);
  }

  if (this->in_power_dip) {
    auto current_time = this->nh->now();
    auto duration = current_time - this->turn_off_trigger_time;
    std::cout << "Triggering turn off maybe" << duration.seconds() << std::endl;
    std::cout << "you have " << (this->ina_data->power_low_time - duration.seconds()) << "s left"
              << std::endl;
    if (duration.seconds() > this->ina_data->power_low_time) {
      std::cout << "Turning off" << std::endl;
      // this->tmx->shutdown();
      this->shutdown_robot();
    }
  }
}

void INA226_sensor::shutdown_robot()
{
  if (this->shutdown_triggered) {
    return;
  }
  this->shutdown_triggered = true;
  std::cout << "Shutting down robot" << std::endl;
  // run shutdown command
  exec("sudo bash -c \"wall 'Shutting down.'\"");  // TODO: check if this works
                                                   // with sudo on a mirte
  exec("sudo shutdown now");
}

void INA226_sensor::shutdown_robot_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) {
    this->shutdown_robot();
  }
  res->success = true;
  res->message = "Shutting down";
}

std::vector<std::shared_ptr<INA226_sensor>> INA226_sensor::get_ina_modules(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser,
  std::shared_ptr<tmx_cpp::Sensors> modules)
{
  std::vector<std::shared_ptr<INA226_sensor>> pca_modules;
  auto pca_data = INA226_data::parse_ina226_data(parser, board);
  for (auto pca : pca_data) {
    std::cout << "ina data" << pca->name << std::endl;
    auto pca_module = std::make_shared<INA226_sensor>(nh, tmx, board, pca->name, modules, pca);
    pca_modules.push_back(pca_module);
  }
  return pca_modules;
}