#pragma once
#include <boost/algorithm/string.hpp>

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

class ModuleData : public DeviceData
{
public:
  /* TODO: DATA*/
  // TODO-NOTE: Added Type
  std::string type;

  ModuleData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
    std::map<std::string, rclcpp::ParameterValue> parameters);

  bool check();
  bool check(std::string module_type);

  static std::string get_device_class() { return "modules"; }
  static std::string get_module_type() { return "no_type"; }
};

// TODO: Redo this for modules so we do not have to dump 3 times
template <class T>
std::vector<typename std::enable_if<std::is_base_of<ModuleData, T>::value, T>::type> parse_all_modules(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board)
{
  auto logger = parser->nh->get_logger();
  const auto device_class = T::get_device_class();
  const auto module_type = boost::to_lower_copy(T::get_module_type());

  std::vector<T> devices;
  for (auto name : parser->get_params_keys(device_class)) {
    auto parameters = parser->get_params_name(parser->build_param_name(device_class, name));

    if (parameters.count("type")) {
      auto supplied_type = get_string(parameters["type"]);
      boost::to_lower(supplied_type);
      if (supplied_type.compare(module_type)){
        RCLCPP_WARN(logger, "Skipping module '%s' since type is not '%s', but '%s'.", name.c_str(), module_type.c_str(), supplied_type.c_str());
        continue;
      }
    } else {
      RCLCPP_ERROR(logger, "Skipping Module '%s' because of missing 'type' field.", name.c_str());
      continue;
    }

    auto data = T(parser, board, name, parameters);

    if (data.check())
      devices.push_back(data);
    else
      RCLCPP_ERROR(
        logger, "%s device \"%s\" is invalid, skipping configuration.",
        device_class.c_str(), name.c_str());
  }

  return devices;
}