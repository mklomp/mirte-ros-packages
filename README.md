# mirte-telemetrix-cpp

```make -k CXX=include-what-you-use CXXFLAGS="-Xiwyu --error_always"```


# Feature matrix:

|                        | ROS_Telemetrix_aio (ros1) | mirte-telemetrix-cpp | remarks |
| ---------------------- | ------------------------- | -------------------- | ------- |
| **Sensors**            |                           |                      |
| Sonar                  | ✅                         | ❓                    |         |
| intensity              | ✅                         | ❓                    |         |
| encoders               | ✅                         | ❌                    |         |
| get pin service        | ✅                         | ❓                    |         |
| **Actuators**          |                           |                      |         |
| oled                   | ✅                         | ❌                    |         |
| servos                 | ✅                         | ❌                    |         |
| keypad                 | ✅                         | ❓                    |         |
| motors                 | ✅                         | ❓                    |         |
| set pin service        | ✅                         | ❓                    |         |
| **Mirte-master parts** |                           |                      |         |
| hiwonder servo         | ✅                         | ✅                    |         |
| pca9685 pwm            | ✅                         | ✅                    |         |
| ina226                 | ✅                         | ❌                    |         |
| imu                    | ✅                         | ❌                    |         |
| ledstrip               | ✅                         | ❌                    |         |
| oled module            | ✅                         | ❌                    |         |

