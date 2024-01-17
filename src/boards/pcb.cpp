#include <mirte-board.hpp>
#include <pcbs/v08.hpp>
Mirte_Board_pcb::Mirte_Board_pcb(
  // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh,
  std::shared_ptr<Mirte_Board> mcu)
// : Mirte_Board(tmx, nh)
{
  this->mcu = mcu;
}

std::map<std::string, int> Mirte_Board_pcb::resolveConnector(std::string connector) {
        
        auto pins = mirte_pico_pcb_map08[connector];
        std::map<std::string, int> resolved_pins;
        for (auto pin : pins) {
          resolved_pins[pin.first] = mcu->resolvePin(pin.second);
        }
        return resolved_pins;
}

int Mirte_Board_pcb::resolvePin(std::string pin) {
  return mcu->resolvePin(pin);
}