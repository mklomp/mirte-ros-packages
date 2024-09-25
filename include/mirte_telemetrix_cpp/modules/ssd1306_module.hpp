#pragma once
#include <filesystem>
#include <optional>

namespace fs = std::filesystem;

#include <tmx_cpp/modules/SSD1306_oled.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/ssd1306_data.hpp>

#include <mirte_msgs/srv/set_oled_file.hpp>
#include <mirte_msgs/srv/set_oled_image.hpp>
#include <mirte_msgs/srv/set_oled_image_legacy.hpp>
#include <mirte_msgs/srv/set_oled_text.hpp>

class SSD1306_module : public Mirte_module
{
public:
  SSD1306_module(
    NodeData node_data, SSD1306Data oled_data, std::shared_ptr<tmx_cpp::Modules> modules);

  SSD1306Data data;
  std::shared_ptr<tmx_cpp::SSD1306_module> ssd1306;

  static std::vector<std::shared_ptr<SSD1306_module>> get_ssd1306_modules(
    NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules);

  bool set_text(std::string text);
  bool set_image(uint8_t width, uint8_t height, uint8_t img_buffer[]);
  bool set_image_from_path(fs::path path);
  bool set_image_from_path(std::string path);

private:
  bool default_image = true;
  bool enabled = true;
  std::optional<std::string> last_text;

  rclcpp::CallbackGroup::SharedPtr oled_access_callback_group;

  rclcpp::Service<mirte_msgs::srv::SetOLEDImageLegacy>::SharedPtr set_oled_service_legacy;

  rclcpp::Service<mirte_msgs::srv::SetOLEDText>::SharedPtr set_oled_text_service;
  rclcpp::Service<mirte_msgs::srv::SetOLEDImage>::SharedPtr set_oled_image_service;
  rclcpp::Service<mirte_msgs::srv::SetOLEDFile>::SharedPtr set_oled_file_service;

  bool prewrite();

  void set_oled_callback_legacy(
    const std::shared_ptr<mirte_msgs::srv::SetOLEDImageLegacy::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetOLEDImageLegacy::Response> res);

  void set_oled_text_callback(
    const std::shared_ptr<mirte_msgs::srv::SetOLEDText::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetOLEDText::Response> res);

  void set_oled_image_callback(
    const std::shared_ptr<mirte_msgs::srv::SetOLEDImage::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetOLEDImage::Response> res);

  void set_oled_file_callback(
    const std::shared_ptr<mirte_msgs::srv::SetOLEDFile::Request> req,
    std::shared_ptr<mirte_msgs::srv::SetOLEDFile::Response> res);
};