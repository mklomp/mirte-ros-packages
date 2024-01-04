#include "mirte-ping.hpp"

Mirte_Ping::Mirte_Ping(TMX &tmx, ros::NodeHandle &nh,
                       std::function<void()> stop_func)
{
  this->tmx = &tmx;
  this->nh = &nh;
  this->ping_thread = std::thread(&Mirte_Ping::ping_task, this);
  this->tmx->add_callback(
      TMX::MESSAGE_IN_TYPE::PONG_REPORT,
      std::bind(&Mirte_Ping::ping_callback, this, std::placeholders::_1));
  this->stop_func = stop_func;
}

void Mirte_Ping::ping_task()
{
  uint8_t num = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (!this->is_stopped)
  {
    num++;
    this->tmx->sendPing(num);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    if ((num - this->last_ping) > 2)
    {
      std::cout << "stop!!!" << (int)((num - this->last_ping)) << std::endl;
      this->stop_func();
    }
  }
}

void Mirte_Ping::ping_callback(std::vector<uint8_t> message)
{
  if (message.size() != 3)
  {
    std::cout << "ping message size != 3" << message.size() << std::endl;
    return;
  }
  if (this->first_magic)
  {
    this->magic = message[1];
    this->first_magic = false;
  }
  if (this->magic != message[1])
  {
    std::cout << "magic changed" << (int)this->magic << "-> " << (int)message[1] << std::endl;
    this->stop_func();
  }
  this->last_ping = message[2];
}

void Mirte_Ping::stop()
{
  this->is_stopped = true;
  this->ping_thread.join();
  this->stop_func = []() {};
}