#include "mirte-actuators.hpp"

Mirte_Actuators::Mirte_Actuators(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board)
{
    this->tmx = &tmx;
    this->nh = &nh;
    this->board = &board;
    XmlRpc::XmlRpcValue actuators;
}