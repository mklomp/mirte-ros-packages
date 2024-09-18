#pragma once

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>
#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

// namespace mirte_telemetrix_cpp {

class Mirte_Sensor : public TelemetrixDevice
{
public:
  virtual void update() override = 0;
  Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins, SensorData data);
};

// }  // namespace mirte_telemetrix_cpp