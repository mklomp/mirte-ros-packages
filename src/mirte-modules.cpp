#include <mirte_telemetrix_cpp/mirte-modules.hpp>
#include <mirte_telemetrix_cpp/util.hpp>

#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>
#include <mirte_telemetrix_cpp/modules/ina226_module.hpp>
#include <mirte_telemetrix_cpp/modules/pca_module.hpp>

Mirte_modules::Mirte_modules(NodeData node_data, std::shared_ptr<Parser> parser)
: nh(node_data.nh), tmx(node_data.tmx), board(node_data.board)
{
  this->module_sys = std::make_shared<tmx_cpp::Modules>(tmx);

  RCLCPP_INFO(nh->get_logger(), "Adding PCA Modules");
  auto pca_mods = PCA_Module::get_pca_modules(node_data, parser, this->module_sys);
  this->modules.insert(this->modules.end(), pca_mods.begin(), pca_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding HiWonder Modules");
  auto hiwonder_mods =
    Hiwonder_bus_module::get_hiwonder_modules(node_data, parser, this->module_sys);
  std::cout << "Adding hiwonder modules" << hiwonder_mods.size() << std::endl;
  this->modules.insert(this->modules.end(), hiwonder_mods.begin(), hiwonder_mods.end());

  this->sensor_sys = std::make_shared<tmx_cpp::Sensors>(tmx);
  RCLCPP_INFO(nh->get_logger(), "Adding INA226 Modules");
  auto ina_mods = INA226_sensor::get_ina_modules(node_data, parser, this->sensor_sys);
  std::cout << "Adding ina modules" << ina_mods.size() << std::endl;

  this->modules.insert(this->modules.end(), ina_mods.begin(), ina_mods.end());
}
