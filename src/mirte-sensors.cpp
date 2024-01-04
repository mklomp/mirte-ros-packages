

#include "mirte-sensors.hpp"
#include <ros/ros.h>
Mirte_Sensors::Mirte_Sensors(TMX &tmx, ros::NodeHandle &nh,
                             Mirte_Board &board)
{
    this->tmx = &tmx;
    this->nh = &nh;
    this->board = &board;
    auto keypads = KeypadMonitor::get_keypad_monitors(nh, tmx, board);
    this->sensors.insert(this->sensors.end(), keypads.begin(), keypads.end());
}
std::vector<KeypadMonitor *>
KeypadMonitor::get_keypad_monitors(ros::NodeHandle &nh, TMX &tmx,
                                   Mirte_Board &board)
{
    // nh->getparam("sensor_port", sensor_port);
    XmlRpc::XmlRpcValue keypads;
    if (!nh.getParam("/mirte/keypad", keypads))
    {
        ROS_ERROR("No keypads found");
        return std::vector<KeypadMonitor *>();
    }
    std::vector<KeypadMonitor *> sensors;
    std::cout << keypads.size() << std::endl;

    for (auto &keypad_it : keypads)
    {
        std::cout << keypad_it.first << std::endl;
        auto keypad = keypad_it.second;
        auto name = keypad_it.first;
        std::cout << "asdf" << std::endl;
        if (!keypad.hasMember("device") || keypad["device"] != "mirte")
        {
            continue;
        }
        if (!(keypad.hasMember("port") || keypad.hasMember("pins")))
        {
            ROS_ERROR("Keypad missing port or name");
            continue;
        }
        auto pins = board.resolvePins(keypad);
        KeypadMonitor *monitor = new KeypadMonitor(tmx, nh, board, pins, name);
        sensors.push_back(monitor);
    }

    return sensors;
    // TODO: schedule periodic publishing
}

Mirte_Sensor::Mirte_Sensor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
                           std::vector<uint8_t> pins, std::string name)
{
    this->tmx = &tmx;
    this->nh = &nh;
    this->pins = pins;
    this->name = name;
    this->board = &board;
}

KeypadMonitor::KeypadMonitor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
                             std::vector<uint8_t> pins, std::string name)
    : Mirte_Sensor(tmx, nh, board, pins, name)
{

    keypad_pub = nh.advertise<mirte_msgs::Keypad>("/mirte/keypad/" + name, 1);
    keypad_pressed_pub =
        nh.advertise<mirte_msgs::Keypad>("/mirte/keypad/" + name + "_pressed", 1);
    keypad_service =
        nh.advertiseService(name + "_get", &KeypadMonitor::keypad_callback, this);
    tmx.setPinMode(pins[0], TMX::PIN_MODES::ANALOG_INPUT, true, 0);
    tmx.add_analog_callback(
        pins[0], [this](auto pin, auto value)
        { this->callback(value); });
}

void KeypadMonitor::callback(uint16_t value)
{
    this->value = value;
    this->publish();
}

bool KeypadMonitor::keypad_callback(mirte_msgs::GetKeypad::Request &req,
                                    mirte_msgs::GetKeypad::Response &res)
{   
    res.data = this->last_debounced_key;
    return true;
}

void KeypadMonitor::publish()
{
    mirte_msgs::Keypad msg;

    std::string key = "";
    auto maxValue = std::pow(2, board->get_adc_bits());
    auto scale = 4096 / maxValue;
    if (this->value < 70 / scale)
    {
        key = "left";
    }
    else if (this->value < 230 / scale)
    {
        key = "up";
    }
    else if (this->value < 410 / scale)
    {
        key = "down";
    }
    else if (this->value < 620 / scale)
    {
        key = "right";
    }
    else if (this->value < 880 / scale)
    {

        key = "enter";
    }
    // # Do some debouncing
    if (this->last_key != key)
    {
        this->last_debounce_time = ros::Time::now();
    }

    std::string debounced_key = "";
    if (ros::Time::now() - this->last_debounce_time > ros::Duration(0.1))
    {
        debounced_key = key;
    }

    // # Publish the last debounced key
    mirte_msgs::Keypad keypad;
    keypad.header = this->get_header();
    keypad.key = debounced_key;
    this->keypad_pub.publish(keypad);

    // # check if we need to send a pressed message
    if (this->last_debounced_key != "" &&
        this->last_debounced_key != debounced_key)
    { // TODO: check this
        mirte_msgs::Keypad pressed;
        pressed.header = this->get_header();
        pressed.key = this->last_debounced_key;
        this->keypad_pressed_pub.publish(pressed);
    }
    this->last_key = key;
    this->last_debounced_key = debounced_key;
}