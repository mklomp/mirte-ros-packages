# API

The motors are controlled with PWM as as service. By default the topic published by the ROS diff_drive_controller (/mobile_base_controller/cmd_vel) will call that service. One has to disable the ROS diff_drive_controller () to manually set the PWM signal by using the service (eg from commandline).
Currently, the default parameters are set to use the Zoef Arduino-nano PCB.

## Subscribed Topics
/mobile_base_controller/cmd_vel (geometry_msgs/Twist)

## Published Topics
Topics published to depend on the parameters given.

/zoef/<name>_distance (sensor_msgs/Range)
/zoef/<name>_encoder (zoef_msgs/Encoder)
/zoef/<name>_intensity (zoef_msgs/Intensity)

## Services
/zoef_navigation/move
/zoef_navigation/turn
/zoef_pymata/set_<name>_motor_pwm
/zoef/get_pin_value
/zoef/set_pin_mode
/zoef/set_pin_value
/zoef_service_api/get_<topic_name>


## Parameters
Default parameters are in config/zoef_base_config.yaml

### Device settings
device/zoef/type (string, default: "zoef")
    The type of board connected. Currently only suppots Zoef. Future support will also include Lego Mindstorms and M-bot.
device/zoef/dev (string, default: "/dev/ttyUSB0")
    The linux device name of the board.

#### Distance sensor
distance/<name>_distance/dev (string, default: "zoef")
distance/<name>_distance/frequency (int, default: 10)
    The frequency to publish new sensor data
distance/<name>_distance/pin (array)
    The (arduino/stm32) pins it is connected to [trigger-pin, echo-pin] (eg [8. 13])

#### Encoder sensor
Note: the encoder sensor does not have a frequency since it uses the interrupt pins
encoder/<name>_encoder/device (string, default: "zoef")
encoder/<name>_encoder/pin (int)
encoder/<name>_encoder/ticks_per_wheel (int, default: 20)

#### Intensity sensor
intensity/<name>_intensity/device (string, default: "zoef")
intensity/<name>_intensity/frequency (int, default: 10)
intensity/<name>_intensity/pin (int)

#### Motor settings
motor/<name>_motor/device (string, default: "zoef")
motor/<name>_motor/pin (array)
     The (arduino/stm32) pins the motor (h-bridge) is connected to. When using 2 pins per motor it will assume MX1919 (both PWM signals). When using 3 pins it will assume L298N (1 PWM, 2 non-PWM)



