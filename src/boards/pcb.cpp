#include <mirte-board.hpp>
#include <pcbs/v08.hpp>
Mirte_Board_pcb::Mirte_Board_pcb(
  std::shared_ptr<Mirte_Board> mcu)
{
  this->mcu = mcu;
}

std::map<std::string, int> Mirte_Board_pcb::resolveConnector(std::string connector)
{

  const auto pins = mirte_pico_pcb_map08.at(connector);
  std::map<std::string, int> resolved_pins;
  for (auto pin : pins) {
    resolved_pins[pin.first] = mcu->resolvePin(pin.second);
  }
  return resolved_pins;
}

int Mirte_Board_pcb::resolvePin(std::string pin)
{
  return mcu->resolvePin(pin);
}
