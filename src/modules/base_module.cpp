#include <mirte_telemetrix_cpp/modules/base_module.hpp>

Mirte_module::Mirte_module(NodeData node_data, std::string name)
: nh(node_data.nh), tmx(node_data.tmx), board(node_data.board), name(name)
{
}