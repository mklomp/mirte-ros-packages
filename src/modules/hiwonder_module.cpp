#include <functional>

#include <rclcpp/callback_group.hpp>

#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>

using namespace std::placeholders;  // for _1, _2, _3...

// hiwonder bus
HiWonderBus_module::HiWonderBus_module(
  NodeData node_data, HiWonderBusData bus_data, std::shared_ptr<tmx_cpp::Modules> modules)
: Mirte_module(
    node_data, {bus_data.tx_pin, bus_data.rx_pin}, (ModuleData)bus_data,
    rclcpp::CallbackGroupType::MutuallyExclusive),
  data(bus_data)
{
  this->logger = this->logger.get_child(data.get_device_class()).get_child(data.name);

  // Create a list of ID's
  std::vector<uint8_t> servo_ids;
  for (auto servo : this->data.servos) servo_ids.push_back(servo->id);

  this->bus = std::make_shared<tmx_cpp::HiwonderServo_module>(
    this->data.uart_port, this->data.rx_pin, this->data.tx_pin, servo_ids,
    std::bind(&HiWonderBus_module::position_cb, this, _1),
    std::bind(&HiWonderBus_module::verify_cb, this, _1, _2),
    std::bind(&HiWonderBus_module::range_cb, this, _1, _2, _3),
    std::bind(&HiWonderBus_module::offset_cb, this, _1, _2));

  modules->add_mod(this->bus);

  auto servo_group = this->data.group_name;
  if (!servo_group.ends_with('/')) servo_group.push_back('/');

  for (auto servo_data : this->data.servos) {
    this->servos.push_back(std::make_shared<Hiwonder_servo>(
      node_data, servo_data, this->bus, servo_group, this->callback_group));
  }

  // Create Bus ROS services
  this->enable_all_servos_service = nh->create_service<std_srvs::srv::SetBool>(
    "servo/" + servo_group + "enable_all_servos",
    std::bind(&HiWonderBus_module::enable_cb, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

// TODO: Make result actually Reflect reality
bool HiWonderBus_module::enable_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  this->bus->set_enabled_all(req->data);
  res->success = true;
  res->message = req->data ? "Enabled" : "Disabled";
  return true;
}

std::vector<std::shared_ptr<HiWonderBus_module>> HiWonderBus_module::get_hiwonder_modules(
  NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules)
{
  std::vector<std::shared_ptr<HiWonderBus_module>> hiwonder_modules;
  auto hiwonder_data = parse_all_modules<HiWonderBusData>(parser, node_data.board);
  for (auto hiwonder : hiwonder_data) {
    auto hiwonder_module = std::make_shared<HiWonderBus_module>(node_data, hiwonder, modules);
    hiwonder_modules.push_back(hiwonder_module);
  }
  return hiwonder_modules;
}

void HiWonderBus_module::position_cb(std::vector<tmx_cpp::HiwonderServo_module::Servo_pos> pos)
{
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
void HiWonderBus_module::verify_cb(int id, bool status)
{
  RCLCPP_INFO(logger, "Servo: %d | status: %s", id, status ? "true" : "false");
  // std::cout << "Servo: " << id << " status: " << status << std::endl;
}

void HiWonderBus_module::range_cb(int id, uint16_t min, uint16_t max)
{
  RCLCPP_INFO(logger, "Servo: %d | range: %u %u", id, min, max);
  // std::cout << "Servo: " << id << " range: " << min << " " << max << std::endl;
}

void HiWonderBus_module::offset_cb(int id, uint16_t offset)
{
  RCLCPP_INFO(logger, "Servo: %d | offset: %u", id, offset);
  // std::cout << "Servo: " << id << " offset: " << offset << std::endl;
}
