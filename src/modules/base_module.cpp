#include <mirte_telemetrix_cpp/modules/base_module.hpp>

Mirte_module::Mirte_module(NodeData node_data, std::vector<pin_t> pins, ModuleData module_data)
: TelemetrixDevice(node_data, pins, (DeviceData)module_data)
{
}