#pragma once
#include <tmx.hpp>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
class Mirte_Board{
    public:
    Mirte_Board(TMX &tmx, ros::NodeHandle &nh);
    TMX *tmx;
    ros::NodeHandle *nh;
    int get_adc_bits() {
        return 12;
    }
    std::vector<uint8_t> resolvePins(XmlRpc::XmlRpcValue keypad);
};