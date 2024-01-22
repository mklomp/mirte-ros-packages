from telemetrix_rpi_pico import telemetrix_rpi_pico
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math
import time


class MyNode(Node):

    def __init__(self):
        super().__init__('mirte_telemetrix', automatically_declare_parameters_from_overrides=True)
        try:
           self.board = telemetrix_rpi_pico.TelemetrixRpiPico()
        except:
           print("no connection, shuttign down")
           exit(0)

        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2 %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_board(self):
        return self.board

    def shutdown(self):
        self.board.shutdown()


rclpy.init()
node = MyNode()


def helper_fnc(test_str, sep, value):
   if sep not in test_str:
     return {test_str: value}
   key, val = test_str.split(sep, 1)
   return {key: helper_fnc(val, sep, value)}

from mergedeep import merge
def parseNestedParameters(params):
   nested_params = {}
   print(params)
   for param_name, param_value in params.items():
      test = helper_fnc(param_name, ".", param_value)
      nested_params = merge(nested_params, test)
      print(test)
   return nested_params

orig_params = node.get_parameters_by_prefix('')
params = parseNestedParameters(orig_params)



# Until we update our own fork of TelemtrixAIO to the renamed pwm calls
# we need to add a simple wrapper
async def set_pin_mode_analog_output(board, pin):
    if board_mapping.get_mcu() == "pico":
        await board.set_pin_mode_pwm_output(pin)
    else:
        await board.set_pin_mode_analog_output(pin)


async def analog_write(board, pin, value):
    if board_mapping.get_mcu() == "pico":
        await board.pwm_write(pin, value)
    else:
        await board.analog_write(pin, value)

# Import ROS message types
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import Range
from mirte_msgs.msg import *

# Import ROS services
from mirte_msgs.srv import *


import mirte_telemetrix.mappings.default
import mirte_telemetrix.mappings.nanoatmega328
import mirte_telemetrix.mappings.pico
import mirte_telemetrix.mappings.blackpill_f103c8
import mirte_telemetrix.mappings.pcb


board_mapping = mirte_telemetrix.mappings.default

devices = params["device"]

if devices["mirte"]["type"].get_parameter_value().string_value == "pcb":
    board_mapping = mappings.pcb
    if "version" in devices["mirte"]:
        if "board" in devices["mirte"]:
            board_mapping.set_version(
                devices["mirte"]["version"].get_parameter_value().string_value,
                devices["mirte"]["board"].get_parameter_value().string_value
            )
        else:
            board_mapping.set_version(devices["mirte"]["version"].get_parameter_value().string_value)

if devices["mirte"]["type"].get_parameter_value().string_value == "breadboard":
    if "board" in devices["mirte"]:
        if devices["mirte"]["board"].get_parameter_value().string_value == "blackpill_f103c8": 
            board_mapping = mirte_telemetrix.mappings.blackpill_f103c8
        elif (devices["mirte"]["board"].get_parameter_value().string_value == "nanoatmega328" or
              devices["mirte"]["board"].get_parameter_value().string_value == "nanoatmega328new" or
              devices["mirte"]["board"].get_parameter_value().string_value == "uno"):  # uno has the same pinout
            board_mapping = mirte_telemetrix.mappings.nanoatmega328
        elif devices["mirte"]["board"].get_parameter_value().string_value == "pico":
            board_mapping = mirte_telemetrix.mappings.pico
        else:
            board_mapping = mirte_telemetrix.mappings.default


def get_pin_numbers(component):
    pins = {}
    if "connector" in component:
        pins = board_mapping.connector_to_pins(component["connector"])
    if "pins" in component:
        pins = component["pins"]
    if "pin" in component:
        pins["pin"] = component["pin"]
    # convert pin naming to numbers
    pin_numbers = {}
    for item in pins:
        pin_value = pins[item].get_parameter_value().integer_value
        if not pin_value:
           pin_value = pins[item].get_parameter_value().string_value
        pin_numbers[item] = board_mapping.pin_name_to_pin_number(pin_value)

    return pin_numbers


# Abstract Sensor class
class SensorMonitor:
    def __init__(self, board, sensor, publisher):
        self.board = board
        self.pins = get_pin_numbers(sensor)
        self.publisher = publisher
        self.max_freq = 10
        if "max_frequency" in sensor:
            self.max_freq = sensor["max_frequency"]
        self.differential = 0
        if "differential" in sensor:
            self.differential = sensor["differential"]
        self.last_publish_time = -1
        self.last_publish_value = {}
        node.get_logger().info(f'Sensor initialized on topic %s (max_freq: %d, differential: %d)', once=True)
#            self.publisher.name,
#            self.max_freq,
#            self.differential,

    def get_header(self):
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        return header

    # NOTE: although there are no async functions in this
    # the function needs to be async since it is called
    # inside a callback of an awaited part of telemetrix
    def publish_imp(self, data):
        self.publisher.publish(data)
        self.last_publish_value = data

    def publish(self, data):
        if self.max_freq == -1:
            self.publish_imp(data)
        else:
            now_millis = int(round(time.time() * 1000))

            # always publish the first message (TODO: and maybe messages that took too long 2x 1/freq?)
            if self.last_publish_time == -1:
                self.publish_imp(data)
                self.last_publish_time = now_millis

            # from then on publish if needed based on max_freq
            if now_millis - self.last_publish_time >= 1000.0 / self.max_freq:
                self.publish_imp(data)
                self.last_publish_time += (
                    1000.0 / self.max_freq
                )  # Note: this should not be set to now_millis. This is due to Nyquist.


class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(
            Range, "/mirte/distance/" + sensor["name"].get_parameter_value().string_value, latching_qos
        )
        srv = node.create_service(
            GetDistance, "/mirte/get_distance_" + sensor["name"].get_parameter_value().string_value, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = Range()

    def get_data(self, req, res):
        res.range = self.last_publish_value.range
        return res

    def start(self):
        #   await self.board.set_scan_delay(100)
        self.board.set_pin_mode_sonar(
            self.pins["trigger"], self.pins["echo"], self.publish_data
        )

    def publish_data(self, data):
        # Although the initialization of this Range message
        # including some of the values could be placed in the
        # constructor for efficiency reasons. This does
        # for some reason not work though.
        range = Range()
        range.radiation_type = range.ULTRASOUND
        range.field_of_view = math.pi * 5
        range.min_range = 0.02
        range.max_range = 1.5
        range.header = self.get_header()
        range.range = float(data[2])
        self.publish(range)



# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(board, device):
    tasks = []
    max_freq = 30
    #if params["device"]["mirte"]["max_frequency"]:
    #    max_freq = params["device"]["mirte"]["max_frequency"].get_parameter_value().integer_value

    # initialze distance sensors
    if ("distance" in params):
        distance_sensors = params["distance"]
        distance_sensors = {
            k: v for k, v in distance_sensors.items() if v["device"].get_parameter_value().string_value == device
        }
        for sensor in distance_sensors:
            distance_sensors[sensor]["max_frequency"] = max_freq
            monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
            monitor.start()






def main(args=None):

    try:
      board = node.get_board()
#      sensors(board, 'mirte')
      rclpy.spin(node)
    except Exception as e:
      print(e)
      print("done")
      #node.shutdown()
    finally:
      node.shutdown()
      node.destroy_node()


if __name__ == '__main__':
    main()
