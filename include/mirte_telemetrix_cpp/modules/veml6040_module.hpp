#pragma once
#include <cstdint>
#include <memory>
#include <shared_mutex>

#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include <tmx_cpp/sensors/VEML6040.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/veml6040_data.hpp>

#include <mirte_msgs/msg/color_hsl.hpp>
#include <mirte_msgs/msg/color_hsl_stamped.hpp>
#include <mirte_msgs/msg/color_rgba_stamped.hpp>
#include <mirte_msgs/msg/color_rgbw.hpp>
#include <mirte_msgs/msg/color_rgbw_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <mirte_msgs/srv/get_color_hsl.hpp>
#include <mirte_msgs/srv/get_color_rgba.hpp>
#include <mirte_msgs/srv/get_color_rgbw.hpp>

class VEML6040_sensor : public Mirte_module {
  public:
    VEML6040_sensor(
      NodeData node_data, VEML6040Data data, std::shared_ptr<tmx_cpp::Sensors> modules);

    VEML6040Data data;
    std::shared_ptr<tmx_cpp::VEML6040_module> veml6040;

    virtual void update() override;
    void data_cb(uint16_t red, uint16_t green, uint16_t blue, uint16_t white);

    static std::vector<std::shared_ptr<VEML6040_sensor>> get_veml6040_modules(
      NodeData node_data, std::shared_ptr<Parser> parser,
      std::shared_ptr<tmx_cpp::Sensors> sensors);

  private:
    // TODO: It might be good to use locking timeouts, however we only lock for a short duration.
    std::shared_mutex msg_mutex;
    mirte_msgs::msg::ColorRGBW last_rgbw;
    std_msgs::msg::ColorRGBA last_rgba;
    mirte_msgs::msg::ColorHSL last_hsl;

    rclcpp::Publisher<mirte_msgs::msg::ColorRGBWStamped>::SharedPtr rgbw_pub;
    rclcpp::Publisher<mirte_msgs::msg::ColorRGBAStamped>::SharedPtr rgba_pub;
    rclcpp::Publisher<mirte_msgs::msg::ColorHSLStamped>::SharedPtr hsl_pub;

    rclcpp::Service<mirte_msgs::srv::GetColorRGBW>::SharedPtr rgbw_service;
    rclcpp::Service<mirte_msgs::srv::GetColorRGBA>::SharedPtr rgba_service;
    rclcpp::Service<mirte_msgs::srv::GetColorHSL>::SharedPtr hsl_service;

    void get_rgbw_service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetColorRGBW::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetColorRGBW::Response> res);
    void get_rgba_service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetColorRGBA::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetColorRGBA::Response> res);
    void get_hsl_service_callback(
      const std::shared_ptr<mirte_msgs::srv::GetColorHSL::Request> req,
      std::shared_ptr<mirte_msgs::srv::GetColorHSL::Response> res);
};
