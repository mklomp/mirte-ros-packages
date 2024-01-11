#pragma once
#include "mirte-actuators.hpp"
#include "mirte-board.hpp"
#include "mirte-ping.hpp"
#include "mirte-sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tmx.hpp>

#include "rclcpp/rclcpp.hpp"

class mirte_node : public rclcpp::Node
{
private:
        /* data */
public:
        mirte_node(/* args */);
        ~mirte_node();
        void start(std::shared_ptr<rclcpp::Node> s_node);
};


