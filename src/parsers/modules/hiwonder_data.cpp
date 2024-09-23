#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder_data.hpp>

HiWonderBusData::HiWonderBusData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters, std::set<std::string> & unused_keys)
: ModuleData(parser, board, name, parameters, unused_keys)
{
  auto logger = parser->nh->get_logger();

  if (unused_keys.erase("connector")) {
    rcpputils::check_true(
      false, (boost::format("No pins tag was supplied to PCA module '%1%' [Connector "
                            "configuration not supported yet.]") %
              name)
               .str());
  } else if (unused_keys.erase("pins")) {
    // FIXME: Shouldn't this be moved under pins?
    if (parameters.count("pins.rx"))
      this->rx_pin = board->resolvePin(get_string(parameters["pins.rx"]));
    if (parameters.count("pins.tx"))
      this->tx_pin = board->resolvePin(get_string(parameters["pins.tx"]));
  } else
    RCLCPP_ERROR(
      logger, "Device %s.%s has no a connector or pins specified. (Connector not supported yet)",
      get_device_class().c_str(), name.c_str());

  this->uart_port = board->resolveUARTPort(this->rx_pin);

  // FIXME: NESTED PARAMS
  if (unused_keys.erase("servos"))
    this->servos = Hiwonder_servo_data::parse_hiwonder_servo_data(
      parser, board, parser->build_param_name(get_device_class(), name));
}

bool HiWonderBusData::check()
{
  return uart_port != 0xFF && tx_pin != (pin_t)-1 && rx_pin != (pin_t)-1 &&
         ModuleData::check(get_module_type());
}