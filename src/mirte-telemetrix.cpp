#include "mirte-actuators.hpp" // for Mirte_Actuators
#include "mirte-board.hpp"     // for Mirte_Board
#include "mirte-node.hpp"
#include "mirte-ping.hpp"          // for Mirte_Ping
#include "mirte-sensors.hpp"       // for Mirte_Sensors
#include "parsers/parsers.hpp"     // for Parser
#include "tmx.hpp"                 // for TMX, TMX::GET_PICO_UNIQUE_ID, TMX...
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
    node->start(node);
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

mirte_node::~mirte_node() { this->s_tmx->stop(); }

void mirte_node::start(std::shared_ptr<rclcpp::Node> s_node) {
  Parser p(s_node);
  auto p_s = std::make_shared<Parser>(p);
  std::shared_ptr<Mirte_Board> s_board = Mirte_Board::create(p_s);
  auto s_tmx = std::make_shared<TMX>("/dev/ttyACM0");
  s_tmx->sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  s_tmx->setScanDelay(10);
  this->actuators =
      std::make_shared<Mirte_Actuators>(s_node, s_tmx, s_board, p_s);
  this->monitor = std::make_shared<Mirte_Sensors>(s_node, s_tmx, s_board, p_s);
  std::cout << "done adding" << std::endl;
  this->ping = std::make_shared<Mirte_Ping>(s_node, s_tmx, [&]() {
    std::cout << "stop" << std::endl;
    rclcpp::shutdown();
  });
}
