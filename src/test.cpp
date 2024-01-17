#include <mirte-board.hpp>
int main()
{
  // Your code here
  Mirte_Board_pico board;//(/*s_tmx, s_node*/);
  std::shared_ptr<Mirte_Board> s_board = std::make_shared<Mirte_Board_pico>(board);
  std::cout << s_board->resolvePin("GP0") << std::endl;
  return 0;
}
