#include "mirte-actuators.hpp"

Mirte_Actuators::Mirte_Actuators(TMX &tmx, ros::NodeHandle &nh,
                                 Mirte_Board &board)
{
    this->tmx = &tmx;
    this->nh = &nh;
    this->board = &board;
    this->actuators = Motor::get_motors(nh, tmx, board);
}

Mirte_Actuator::Mirte_Actuator(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board, std::vector<uint8_t> pins, std::string name)
{
    this->tmx = &tmx;
    this->nh = &nh;
    this->board = &board;
    this->pins = pins;
    this->name = name;
}
std::vector<Mirte_Actuator *> Motor::get_motors(ros::NodeHandle &nh, TMX &tmx,
                                                Mirte_Board &board)
{
    std::vector<Mirte_Actuator *> motors;
    XmlRpc::XmlRpcValue motors_config;
    nh.getParam("/mirte/motor", motors_config);
    for (auto &motor_it : motors_config)
    {
        std::cout << motor_it.first << std::endl;
        XmlRpc::XmlRpcValue motor_config = motor_it.second;
        ROS_ASSERT(motor_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        std::string type = motor_config["type"];
        std::string name = motor_it.first;
        std::vector<uint8_t> pins = board.resolvePins(motor_config["pins"]);
        if (type == "pp")
        {
            motors.push_back(new PPMotor(tmx, nh, board, pins, name));
        }
        else if (type == "dp")
        {
            motors.push_back(new DPMotor(tmx, nh, board, pins, name));
        }
        else
        {
            ROS_ERROR("Unknown motor type: %s", type.c_str());
        }
    }
    return motors;
}
bool Motor::service_callback(mirte_msgs::SetMotorSpeed::Request &req,
                             mirte_msgs::SetMotorSpeed::Response &res)
{
    this->set_speed(req.speed);
    // res.success = true;
    return true;
}

void Motor::motor_callback(const std_msgs::Int32 &msg)
{
    this->set_speed(msg.data);
}

Motor::Motor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
             std::vector<uint8_t> pins, std::string name) : Mirte_Actuator(tmx, nh, board, pins, name)
{
    motor_service = nh.advertiseService("/mirte/set_" + this->name + "_speed", &Motor::service_callback, this);
    ros_client = nh.subscribe("/mirte/motor_" + this->name + "_speed", 1000,
                              &Motor::motor_callback,
                              this);
}

DPMotor::DPMotor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
                 std::vector<uint8_t> pins, std::string name)
    : Motor(tmx, nh, board, pins, name)
{
    ROS_ASSERT(pins.size() == 2);
    tmx.setPinMode(pins[0], TMX::PIN_MODES::DIGITAL_OUTPUT);
    tmx.setPinMode(pins[1], TMX::PIN_MODES::PWM_OUTPUT);
    // motor_service = nh.advertiseService(name, &Motor::service_callback, this);
    // ros_client = nh.subscribe<mirte_msgs::SetMotorSpeed>(name, 1000,
    //                                                      &Motor::motor_callback,
    //                                                      this);
}

void DPMotor::set_speed(int speed)
{
    int32_t speed_ = speed * std::pow(2, 16) / 100;
    tmx->pwmWrite(pins[1], speed > 0 ? speed_ : -speed_);
    std::cout << "1:" << std::dec << speed << std::endl;
    tmx->digitalWrite(pins[0], speed > 0 ? 1 : 0);
    std::cout << "2:" << std::dec << (speed > 0 ? 1 : 0) << std::endl;
    std::cout << "Setting speed to " << std::dec << speed << std::endl;
    // tmx->set_digital_pwm(pins[0], speed);
}

PPMotor::PPMotor(TMX &tmx, ros::NodeHandle &nh, Mirte_Board &board,
                 std::vector<uint8_t> pins, std::string name)
    : Motor(tmx, nh, board, pins, name)
{
}
void PPMotor::set_speed(int speed)
{
    int32_t speed_ = speed * std::pow(2, 16) / 100;
    tmx->pwmWrite(pins[0], speed > 0 ? speed_ : 0);
    tmx->pwmWrite(pins[1], speed < 0 ? -speed_ : 0);
    std::cout << "1:" << std::dec << (speed < 0 ? -speed_ : 0) << std::endl;
    std::cout << "2:" << std::dec << (speed > 0 ? speed_ : 0) << std::endl;
    std::cout << "Setting speed to " << std::dec << speed << std::endl;

    std::cout << "PP Setting speed to " << std::dec << speed << std::endl;
    // tmx->set_pwm(pins[0], speed);
}