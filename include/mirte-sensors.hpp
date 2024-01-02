#pragma once
#include <tmx.hpp>
#include <ros/ros.h>
#include <mirte_msgs/GetKeypad.h>
#include <mirte_msgs/Keypad.h>
#include "mirte-board.hpp"
class Mirte_Sensor;
class Mirte_Sensors
{
public:
    Mirte_Sensors(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board);
    TMX *tmx;
    ros::NodeHandle *nh;
    Mirte_Board *board;
    std::vector<Mirte_Sensor *> sensors;
    std::vector<uint8_t> resolvePins(XmlRpc::XmlRpcValue);
};

class Mirte_Sensor
{
public:
    TMX *tmx;
    ros::NodeHandle *nh;
    Mirte_Board *board;
    std::vector<uint8_t> pins;
    virtual void publish() = 0;
    std::string name;
    auto get_header()
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        return header;
    }
    Mirte_Sensor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board, std::vector<uint8_t> pins, std::string name);
};

class KeypadMonitor : public Mirte_Sensor
{
public:
    KeypadMonitor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board, std::vector<uint8_t> pins, std::string name);
    void publish();
    ros::Publisher keypad_pub;
    ros::Publisher keypad_pressed_pub;
    ros::ServiceServer keypad_service;
    bool keypad_callback(mirte_msgs::GetKeypad::Request &req, mirte_msgs::GetKeypad::Response &res);
    void callback(uint16_t value);
    uint16_t value;
    std::string last_key;
    decltype(ros::Time::now()) last_debounce_time = ros::Time::now();
    std::string last_debounced_key;

    static std::vector<KeypadMonitor *> get_keypad_monitors(ros::NodeHandle &nh, TMX &tmx, Mirte_Board &board);
};