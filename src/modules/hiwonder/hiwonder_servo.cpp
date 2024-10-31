#include <functional>

#include <mirte_telemetrix_cpp/modules/hiwonder/hiwonder_servo.hpp>

using namespace std::placeholders;

// NOTE: Uses a timer per servo instead of the device timer of the bus

// TODO: Maybe make it inherit from Servo
Hiwonder_servo::Hiwonder_servo(
  NodeData node_data, std::shared_ptr<HiWonderServoData> servo_data,
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod, std::string servo_group,
  DeviceData::DeviceDuration duration, rclcpp::CallbackGroup::SharedPtr callback_group)
: servo_data(servo_data), bus_mod(bus_mod), nh(node_data.nh)
{
  auto logger = nh->get_logger();

  if (!this->bus_mod->verify_id(this->servo_data->id))
    RCLCPP_ERROR(
      logger, "HiWonder Servo '%s' ID not present. [Expected ID %d, but was not found]",
      this->servo_data->name.c_str(), this->servo_data->id);

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
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
    "servo/" + servo_group + this->servo_data->name + "/set_enable",
    std::bind(&Hiwonder_servo::enable_service_callback, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create angle service
  this->angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
    "servo/" + servo_group + this->servo_data->name + "/set_angle",
    std::bind(&Hiwonder_servo::set_angle_service_callback, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create range service
  this->range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
    "servo/" + servo_group + this->servo_data->name + "/get_range",
    std::bind(&Hiwonder_servo::get_range_service_callback, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create publisher
  // Use default QoS for sensor publishers as specified in REP2003
  this->position_pub = nh->create_publisher<mirte_msgs::msg::ServoPosition>(
    "servo/" + servo_group + this->servo_data->name + "/position", rclcpp::SystemDefaultsQoS());

  // last_msg = std::make_shared<mirte_msgs::msg::ServoPosition>();
  // last_msg->header.frame_id = this->servo_data->frame_id;

  // TODO: Maybe add to a separate callbackgroup?
  // Currently overpublishing slightly
  this->servo_timer = nh->create_wall_timer(
    duration,
    [this]() {
      // this->last_msg->header.stamp = this->nh->now();
      // auto msg = std::make_unique<mirte_msgs::msg::ServoPosition>(*this->last_msg);
      mirte_msgs::msg::ServoPosition msg;
      msg.header = this->get_header();
      msg.angle = this->last_angle;
      msg.raw = this->last_raw;
      this->position_pub->publish(msg);
    },
    callback_group);
}

// TODO: Add update en fix time
void Hiwonder_servo::position_cb(tmx_cpp::HiwonderServo_module::Servo_pos & pos)
{
  last_angle = calc_angle_in(pos.angle);
  last_raw = pos.angle;
  // last_msg->angle = last_angle;
  // last_msg->raw = pos.angle;
  // last_msg->header.stamp = nh->now();
  mirte_msgs::msg::ServoPosition msg;
  msg.header = get_header();
  msg.angle = last_angle;
  msg.raw = last_raw;
  // RCLCPP_INFO_STREAM(nh->get_logger(), "M: "  << mirte_msgs::msg::to_yaml(msg, true));
  this->position_pub->publish(msg);
  servo_timer->reset();
}

bool Hiwonder_servo::enable_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  res->success = this->bus_mod->set_enable_servo(this->servo_data->id, req->data);
  res->message = req->data ? "Enabled" : "Disabled";
  return true;
}

void Hiwonder_servo::set_angle_service_callback(
  const std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetServoAngle::Response> res)
{
  float angle = req->angle;
  bool is_degrees = req->degrees;

  if (is_degrees == mirte_msgs::srv::SetServoAngle::Request::DEGREES) {
    angle = angle * (std::numbers::pi / 180.0);
  }

  if (angle > servo_data->max_angle_in || angle < servo_data->min_angle_in) {
    RCLCPP_WARN(
      nh->get_logger(),
      "The provided angle is out of range. Angle %.3f radians was requested, but range is [%.3f, "
      "%.3f]",
      angle, servo_data->min_angle_in, servo_data->max_angle_in);
    res->status = false;
    return;
  }

  auto angle_out = calc_angle_out(angle);
  res->status = this->bus_mod->set_single_servo(this->servo_data->id, angle_out, 100);
}

bool Hiwonder_servo::get_range_service_callback(
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

std_msgs::msg::Header Hiwonder_servo::get_header()
{
  std_msgs::msg::Header header;
  header.stamp = nh->now();
  header.frame_id = this->servo_data->frame_id;

  return header;
}