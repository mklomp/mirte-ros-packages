#include "mirte_telemetrix_cpp/mirte-actuators.hpp" // for Mirte_Actuators
#include "mirte_telemetrix_cpp/mirte-board.hpp"     // for Mirte_Board
#include "mirte_telemetrix_cpp/mirte-modules.hpp"
#include "mirte_telemetrix_cpp/mirte-node.hpp"
#include "mirte_telemetrix_cpp/mirte-ping.hpp"      // for Mirte_Ping
#include "mirte_telemetrix_cpp/mirte-sensors.hpp"   // for Mirte_Sensors
#include "mirte_telemetrix_cpp/parsers/parsers.hpp" // for Parser
#include "tmx_cpp/tmx.hpp"             // for TMX, TMX::GET_PICO_UNIQUE_ID, TMX...
#include "mirte_telemetrix_cpp/util.hpp"
#include <exception>               // for exception
#include <iostream>                // for operator<<, endl, basic_ostream
#include <memory>                  // for make_shared, shared_ptr, __shared...
#include <rclcpp/executors.hpp>    // for spin
#include <rclcpp/node.hpp>         // for Node
#include <rclcpp/node_options.hpp> // for NodeOptions
#include <rclcpp/utilities.hpp>    // for shutdown, init
#include <stdint.h>                // for uint8_t
int main(int argc, char **argv) {
  // Initialize the ROS node
  try {
    rclcpp::init(argc, argv);

    // Spin the ROS node
    auto node = std::make_shared<mirte_node>();
    if (!node->start(node)) {
      return 0;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();

  } catch (std::exception &e) {
    std::cout << "Exception" << e.what() << std::endl;
  }
  return 0;
}

mirte_node::mirte_node(/* args */)
    : rclcpp::Node("mirte_telemetrix_node",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {

}

mirte_node::~mirte_node() {
  if (this->s_tmx) {
    this->s_tmx->stop();
  }
}

bool mirte_node::start(std::shared_ptr<rclcpp::Node> s_node) {
  Parser p(s_node);
  auto p_s = std::make_shared<Parser>(p);
  std::shared_ptr<Mirte_Board> s_board = Mirte_Board::create(p_s);
  auto ports = tmx_cpp::TMX::get_available_ports();
  decltype(ports) available_ports;
  for (auto& port : ports) {
    // maybe move this to a function in tmx
    std::cout << "try " << port.port_name << std::endl;
    if(!tmx_cpp::TMX::is_accepted_port(port)) {
      continue;
    }
    if (!tmx_cpp::TMX::check_port(port.port_name)) {
      continue;
    }
    std::cout << "found " << port.port_name << std::endl;
    available_ports.push_back(port);
  }
  if (available_ports.size() == 0) {
    std::cout << "No ports available" << std::endl;
    // FIXME: REMOVE DEBUG HACK
    // rclcpp::spin(s_node);
    rclcpp::shutdown();
    return false;
  }
  if (available_ports.size() > 1) {
    std::cout << "More than one port available, picking the first one"
              << std::endl;
  }

  if(true) {
    auto mcu_id = 12; // TODO: add to parsing and config
    bool found = false;
    for(auto& port : available_ports) {
      auto id = tmx_cpp::TMX::get_id(port);
      std::cout << "ID: " << std::hex << (int)id << std::endl;
      if(id == 0xff) { // default flash value is 0xff, so no id set
        auto check = tmx_cpp::TMX::set_id(port, mcu_id);
        if(!check) {
          std::cout << "Failed to set MCU ID" << std::endl;
          rclcpp::shutdown();
          return false;
        } else {
          id = mcu_id;
        }
      }
      if(id == mcu_id) {
        available_ports = {port};
        found = true;
        break;
      }
    }
    if(!found) {
      std::cout << "No port with MCU ID " << mcu_id << " found" << std::endl;
      rclcpp::shutdown();
      return false;
    }
  }


  auto s_tmx = std::make_shared<tmx_cpp::TMX>(available_ports[0].port_name);
  s_tmx->sendMessage(tmx_cpp::TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  s_tmx->setScanDelay(10);
  this->actuators =
      std::make_shared<Mirte_Actuators>(s_node, s_tmx, s_board, p_s);
  this->monitor = std::make_shared<Mirte_Sensors>(s_node, s_tmx, s_board, p_s);
  this->modules = std::make_shared<Mirte_modules>(s_node, s_tmx, s_board, p_s);
  std::cout << "done adding" << std::endl;
  this->ping = std::make_shared<Mirte_Ping>(s_node, s_tmx, [&]() {
    std::cout << "stop" << std::endl;
    rclcpp::shutdown();
  });
  return true;
}
