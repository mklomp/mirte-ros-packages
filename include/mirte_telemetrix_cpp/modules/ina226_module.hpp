#pragma once

#include <tmx_cpp/sensors/INA226.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/ina226_data.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

class INA226_sensor : public Mirte_module {
  public:
    INA226_sensor(
      NodeData node_data, INA226Data ina_data, std::shared_ptr<tmx_cpp::Sensors> modules);

    INA226Data data;
    std::shared_ptr<tmx_cpp::INA226_module> ina226;

    // virtual void update() override;

    void data_cb(float voltage, float current);

    float calc_soc(float voltage);
    void integrate_usage(float current);
    void check_soc(float voltage, float current);

    void shutdown_robot();
    void shutdown_robot_cb(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    static std::vector<std::shared_ptr<INA226_sensor>> get_ina_modules(
      NodeData node_data, std::shared_ptr<Parser> parser,
      std::shared_ptr<tmx_cpp::Sensors> sensors);

  private:
    float total_used_mAh = 0;
    rclcpp::Time used_time = rclcpp::Time(0, 0);
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr used_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;

    bool enable_turn_off = false;
    bool shutdown_triggered = false;
    rclcpp::Time turn_off_trigger_time = rclcpp::Time(0, 0);
    bool in_power_dip = false;
    rclcpp::Time turn_off_time = rclcpp::Time(0, 0);
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shutdown_service;

    float voltage_ = 0;

#ifdef WITH_GPIO
    rclcpp::TimerBase::SharedPtr battery_led_timer;
    void battery_led_timer_callback();
#endif
};