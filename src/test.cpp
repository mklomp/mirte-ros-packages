#include <mirte_telemetrix_cpp/mirte-board.hpp>
int main() {
  // Your code here
  Mirte_Board_pico board; //(/*s_tmx, s_node*/);
  Mirte_Board_pcb pcb(std::make_shared<Mirte_Board_pico>(board), "v06");
  std::shared_ptr<Mirte_Board> s_board = std::make_shared<Mirte_Board_pcb>(pcb);
  std::cout << s_board->resolvePin("GP0") << std::endl;
  auto x = s_board->resolveConnector("MC2-B");
  for (auto i : x) {
    std::cout << i.first << " " << i.second << std::endl;
  }
  return 0;
}
