#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

Mirte_Sensor::Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins, SensorData data)
: TelemetrixDevice(node_data, pins, (DeviceData)data)
{
}