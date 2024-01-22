#include <mirte-node.hpp>
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

mirte_node::~mirte_node() {}

void mirte_node::start(std::shared_ptr<rclcpp::Node> s_node) {
  Parser p(s_node);
  // parse_servo_data(s_node);
  auto p_s = std::make_shared<Parser>(p);
  std::shared_ptr<Mirte_Board> s_board = Mirte_Board::create(p_s);
  auto s = s_board->resolveConnector("LED");
  std::cout << "LED" << std::endl;
  for (auto i : s) {
    std::cout << i.first << " " << i.second << std::endl;
  }
  return;
  auto s_tmx = std::make_shared<TMX>("/dev/null");
  s_tmx->sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  s_tmx->setScanDelay(10);
  // auto s_node = std::make_shared<rclcpp::Node>(this);
  // Your code here
  Mirte_Sensors monitor(s_node, s_tmx, s_board, p_s);
  Mirte_Actuators actuators(s_node, s_tmx, s_board, p_s);

  // Mirte_Ping ping( s_node,s_tmx, [&]() {
  //     std::cout << "stop" << std::endl;
  //     rclcpp::shutdown();
  //   });
}
