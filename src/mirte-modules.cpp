#include <mirte_telemetrix_cpp/mirte-modules.hpp>
#include <mirte_telemetrix_cpp/util.hpp>

#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>
#include <mirte_telemetrix_cpp/modules/ina226_module.hpp>
#include <mirte_telemetrix_cpp/modules/pca_module.hpp>

Mirte_modules::Mirte_modules(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx,
  std::shared_ptr<Mirte_Board> board, std::shared_ptr<Parser> parser)
: nh(nh), tmx(tmx), board(board)
{
  this->module_sys = std::make_shared<tmx_cpp::Modules>(tmx);
  auto pca_mods = PCA_Module::get_pca_modules(nh, tmx, board, parser, this->module_sys);
  this->modules.insert(this->modules.end(), pca_mods.begin(), pca_mods.end());

  auto hiwonder_mods =
    Hiwonder_bus_module::get_hiwonder_modules(nh, tmx, board, parser, this->module_sys);
  std::cout << "Adding hiwonder modules" << hiwonder_mods.size() << std::endl;
  this->modules.insert(this->modules.end(), hiwonder_mods.begin(), hiwonder_mods.end());

  this->sensor_sys = std::make_shared<tmx_cpp::Sensors>(tmx);
  auto ina_mods = INA226_sensor::get_ina_modules(nh, tmx, board, parser, this->sensor_sys);
  std::cout << "Adding ina modules" << ina_mods.size() << std::endl;

  this->modules.insert(this->modules.end(), ina_mods.begin(), ina_mods.end());
}
