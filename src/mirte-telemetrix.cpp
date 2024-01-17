#include <mirte-node.hpp>
int main(int argc, char ** argv)
{
  // Initialize the ROS node
  try {
    rclcpp::init(argc, argv);

    // Spin the ROS node
    auto node = std::make_shared<mirte_node>();
    node->start(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

  } catch (std::exception & e) {
    std::cout << "Exception" << e.what() << std::endl;
  }
  return 0;
}


mirte_node::mirte_node(/* args */)
: rclcpp::Node("mirte_telemetrix_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{

}

mirte_node::~mirte_node()
{
}

void mirte_node::start(std::shared_ptr<rclcpp::Node> s_node)
{
  Parser p(s_node);
  // parse_servo_data(s_node);
  auto s = p.get_params_name("servo");
  for (auto & servo_it : s) {
    std::cout << servo_it.first << std::endl;
  }
  std::cout << "2"<<std::endl;
  auto s2 = p.get_params_keys("servo");
  for (auto & servo_it : s2) {
    std::cout << servo_it << std::endl;
  }
  auto s_tmx = std::make_shared<TMX>("/dev/null");
  s_tmx->sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  s_tmx->setScanDelay(10);
  // auto s_node = std::make_shared<rclcpp::Node>(this);
  // Your code here
  Mirte_Board_pico board(s_tmx, s_node);
  auto s_board = std::make_shared<Mirte_Board>((Mirte_Board)board);
  Mirte_Sensors monitor(s_node, s_tmx, s_board);
  Mirte_Actuators actuators(s_node, s_tmx, s_board, std::make_shared<Parser>(p));

  // Mirte_Ping ping( s_node,s_tmx, [&]() {
  //     std::cout << "stop" << std::endl;
  //     rclcpp::shutdown();
  //   });
}
