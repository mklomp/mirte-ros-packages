#include <functional>

#include <mirte_telemetrix_cpp/modules/hiwonder/hiwonder_servo.hpp>

using namespace std::placeholders;

// TODO: Maybe make it inherit from Servo
Hiwonder_servo::Hiwonder_servo(
  NodeData node_data, std::shared_ptr<HiWonderServoData> servo_data,
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod, std::string servo_group,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  auto logger = node_data.nh->get_logger();

  this->servo_data = servo_data;
  this->bus_mod = bus_mod;

  if (!this->bus_mod->verify_id(this->servo_data->id))
    RCLCPP_ERROR(
      logger, "HiWonder Servo '%s' ID not present. [Expected ID %d, but was not found]",
      this->servo_data->name, this->servo_data->id);

  auto range = this->bus_mod->get_range(servo_data->id);
  assert(range.has_value());
  auto [lower, upper] = range.value();
  if (lower != this->servo_data->min_angle_out)
    RCLCPP_WARN(
      logger,
      "HiWonder Servo '%s' lower range does not match the config. [Expected %d , Actual %d]",
      this->servo_data->name.c_str(), this->servo_data->min_angle_out, lower);

  if (upper != this->servo_data->max_angle_out)
    RCLCPP_WARN(
      logger,
      "HiWonder Servo '%s' upper range does not match the config. [Expected %d , Actual %d]",
      this->servo_data->name.c_str(), this->servo_data->max_angle_out, upper);

  auto home = this->bus_mod->get_offset(servo_data->id);
  assert(home.has_value());
  auto home_val = home.value();
  if (home_val != this->servo_data->home_out)
    RCLCPP_WARN(
      logger, "HiWonder Servo '%s' home does not match the config. [Expected %d , Actual %d]",
      this->servo_data->name.c_str(), this->servo_data->home_out, home_val);

  // create enable service
  this->enable_service = node_data.nh->create_service<std_srvs::srv::SetBool>(
    "servo/" + servo_group + this->servo_data->name + "/set_enable",
    std::bind(&Hiwonder_servo::enable_cb, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create angle service
  this->angle_service = node_data.nh->create_service<mirte_msgs::srv::SetServoAngle>(
    "servo/" + servo_group + this->servo_data->name + "/set_angle",
    std::bind(&Hiwonder_servo::angle_cb, this, _1, _2), rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group);

  // create range service
  this->range_service = node_data.nh->create_service<mirte_msgs::srv::GetServoRange>(
    "servo/" + servo_group + this->servo_data->name + "/get_range",
    std::bind(&Hiwonder_servo::range_cb, this, _1, _2), rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group);

  // create publisher
  // Use default QoS for sensor publishers as specified in REP2003
  this->position_pub = node_data.nh->create_publisher<mirte_msgs::msg::ServoPosition>(
    "servo/" + servo_group + this->servo_data->name + "/position", rclcpp::SystemDefaultsQoS());
}

void Hiwonder_servo::position_cb(tmx_cpp::HiwonderServo_module::Servo_pos & pos)
{
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
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  this->bus_mod->set_enable_servo(this->servo_data->id, req->data);
  res->success = true;
  res->message = req->data ? "Enabled" : "Disabled";
  return true;
}

// FIXME: This does not rescpect degrees or radians
bool Hiwonder_servo::angle_cb(
  const std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetServoAngle::Response> res)
{
  auto angle = calc_angle_out(req->angle);
  this->bus_mod->set_single_servo(this->servo_data->id, angle, 100);
  res->status = true;
  return true;
}

bool Hiwonder_servo::range_cb(
  const std::shared_ptr<mirte_msgs::srv::GetServoRange::Request> req,
  std::shared_ptr<mirte_msgs::srv::GetServoRange::Response> res)
{
  res->min = this->servo_data->min_angle_in;
  res->max = this->servo_data->max_angle_in;
  return true;
}

template <typename T>
T scale(T x, T in_min, T in_max, T out_min, T out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Hiwonder_servo::calc_angle_out(float angle)
{
  float angle_out = scale<float>(
    angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
    this->servo_data->min_angle_out, this->servo_data->max_angle_out);
  if (this->servo_data->invert) {
    angle_out = scale<float>(
      angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
      this->servo_data->max_angle_out, this->servo_data->min_angle_out);
  }
  return std::max(
    std::min((int)angle_out, this->servo_data->max_angle_out), this->servo_data->min_angle_out);
}

float Hiwonder_servo::calc_angle_in(uint16_t angle)
{
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
