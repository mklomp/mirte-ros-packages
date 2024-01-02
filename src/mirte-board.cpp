#include <mirte-board.hpp>
#include <xmlrpcpp/XmlRpcValue.h>
Mirte_Board::Mirte_Board(TMX &tmx, ros::NodeHandle &nh)
{
    this->tmx = &tmx;
    this->nh = &nh;
}

std::vector<uint8_t> Mirte_Board::resolvePins(XmlRpc::XmlRpcValue keypad)
{
    std::vector<uint8_t> pins;
    if (keypad.hasMember("port"))
    {
        pins.push_back(1);
    }
    if (keypad.hasMember("pin"))
    {
        pins.push_back(2);
    }
    return pins;
}
