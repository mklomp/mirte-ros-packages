#include <mirte_telemetrix_cpp/modules/base_module.hpp>

Mirte_module::Mirte_module(
  std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<tmx_cpp::TMX> tmx, std::shared_ptr<Mirte_Board> board,
  std::string name)
: nh(nh), tmx(tmx), board(board), name(name)
{
}