#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder_data.hpp>

HiWonderBusData::HiWonderBusData(
  std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board, std::string name,
  std::map<std::string, rclcpp::ParameterValue> parameters)
: ModuleData(parser, board, name, parameters)
{
  if (parameters.count("connector")) {
    rcpputils::check_true(
      false, (boost::format("No {rx/tx}_pin tag was supplied to PCA module '%1%' [Connector "
                            "configuration not supported yet.]") %
              name)
               .str());
  } else {
    // FIXME: Shouldn't this be moved under pins?
    if (parameters.count("rx_pin"))
      this->rx_pin = board->resolvePin(get_string(parameters["rx_pin"]));
    if (parameters.count("tx_pin"))
      this->tx_pin = board->resolvePin(get_string(parameters["tx_pin"]));
  }
  if (parameters.count("uart")) {
    this->uart_port = parameters["uart"].get<uint8_t>();

    // FIXME: Why this port check?
    rcpputils::check_true(
      // Either 0 or 1
      this->uart_port <= 1,
      (boost::format("Invalid uart port %1% specified for module %2%") % this->uart_port % name)
        .str());
  }

  if (parameters.count("servos"))
    this->servos = Hiwonder_servo_data::parse_hiwonder_servo_data(
      parser, board, parser->build_param_name(get_device_class(), name));
}

bool HiWonderBusData::check()
{
  return uart_port <= 1 && tx_pin != (pin_t)-1 && rx_pin != (pin_t)-1 &&
         ModuleData::check(get_module_type());
}