#!/usr/bin/env python3.8
import asyncio
import os, os.path
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import signal
import io
from inspect import signature
from tmx_pico_aio import tmx_pico_aio
from telemetrix_aio import telemetrix_aio


rclpy.init(args=sys.argv)

class MyNode(Node):

    def __init__(self):
       super().__init__('mirte_telemetrix', automatically_declare_parameters_from_overrides=True)

    def on_shutdown(self):
       print("doing this")

#node = rclpy.create_node('mirte_telemetrix', automatically_declare_parameters_from_overrides=True)
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

from bitstring import BitArray
import textwrap

from PIL import Image, ImageDraw, ImageFont

# Currently loading the PIL default font, which is
# monospace, so works with python textwrap
font = ImageFont.load_default()

from adafruit_ssd1306 import _SSD1306


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
        self.loop = asyncio.get_event_loop()
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
    async def publish_imp(self, data):
        self.publisher.publish(data)
        self.last_publish_value = data

    async def publish(self, data):
        if self.max_freq == -1:
            await self.publish_imp(data)
        else:
            now_millis = int(round(time.time() * 1000))

            # always publish the first message (TODO: and maybe messages that took too long 2x 1/freq?)
            if self.last_publish_time == -1:
                await self.publish_imp(data)
                self.last_publish_time = now_millis

            # from then on publish if needed based on max_freq
            if now_millis - self.last_publish_time >= 1000.0 / self.max_freq:
                await self.publish_imp(data)
                self.last_publish_time += (
                    1000.0 / self.max_freq
                )  # Note: this should not be set to now_millis. This is due to Nyquist.


class KeypadMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = node.create_publisher(Keypad, "/mirte/keypad/" + sensor["name"].get_parameter_value().string_value, 10)
        srv = node.create_service(
            GetKeypad, "/mirte/get_keypad_" + sensor["name"].get_parameter_value().string_value, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_debounce_time = 0
        self.last_key = ""
        self.last_debounced_key = ""
        self.pressed_publisher = node.create_publisher(
            Keypad, "/mirte/keypad/" + sensor["name"].get_parameter_value().string_value + "_pressed", 10
        )
        self.last_publish_value = Keypad()

    def get_data(self, req, res):
        res.data = self.last_publish_value.key
        return res

    async def start(self):
        await self.board.set_pin_mode_analog_input(
            self.pins["pin"] - board_mapping.get_analog_offset(),
            differential=self.differential,
            callback=self.publish_data,
        )

    async def publish_data(self, data):
        # Determine the key that is pressed
        # TODO: these values were found on a 12 bits adc, and
        # added a scaling for the actual bits used. We could
        # calculate this with the R values used.
        key = ""
        if data[2] < 70 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "left"
        elif data[2] < 230 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "up"
        elif data[2] < 410 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "down"
        elif data[2] < 620 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "right"
        elif data[2] < 880 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "enter"

        # Do some debouncing
        if self.last_key is not key:
            self.last_debounce_time = data[3]

        debounced_key = ""
        if data[3] - self.last_debounce_time > 0.1:
            debounced_key = key

        # Publish the last debounced key
        keypad = Keypad()
        keypad.header = self.get_header()
        keypad.key = debounced_key
        await self.publish(keypad)

        # check if we need to send a pressed message
        if (self.last_debounced_key != "") and (
            self.last_debounced_key is not debounced_key
        ):
            pressed = Keypad()
            pressed.header = self.get_header()
            pressed.key = self.last_debounced_key
            self.pressed_publisher.publish(pressed)

        self.last_key = key
        self.last_debounced_key = debounced_key


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

    async def start(self):
        #   await self.board.set_scan_delay(100)
        await self.board.set_pin_mode_sonar(
            self.pins["trigger"], self.pins["echo"], self.publish_data
        )

    async def publish_data(self, data):
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
        await self.publish(range)


class DigitalIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(
            IntensityDigital,
            "/mirte/intensity/" + sensor["name"].get_parameter_value().string_value + "_digital",
            latching_qos
        )
        srv = node.create_service(
            GetIntensityDigital,
            "/mirte/get_intensity_" + sensor["name"].get_parameter_value().string_value + "_digital",
            self.get_data,
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = IntensityDigital()

    def get_data(self, req, res):
        res.data = self.last_publish_value.value
        return res

    async def start(self):
        await self.board.set_pin_mode_digital_input(
            self.pins["digital"], callback=self.publish_data
        )

    async def publish_data(self, data):
        intensity = IntensityDigital()
        intensity.header = self.get_header()
        intensity.value = bool(data[2])
        await self.publish(intensity)


class AnalogIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = node.create_publisher(
            Intensity, "/mirte/intensity/" + sensor["name"].get_parameter_value().string_value, 10
        )
        srv = node.create_service(
            GetIntensity, "/mirte/get_intensity_" + sensor["name"].get_parameter_value().string_value, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = Intensity()

    def get_data(self, req, res):
        res.data = self.last_publish_value.value
        return res

    async def start(self):
        await self.board.set_pin_mode_analog_input(
            self.pins["analog"] - board_mapping.get_analog_offset(),
            differential=self.differential,
            callback=self.publish_data,
        )

    async def publish_data(self, data):
        intensity = Intensity()
        intensity.header = self.get_header()
        intensity.value = data[2]
        await self.publish(intensity)


class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(
            Encoder, "/mirte/encoder/" + sensor["name"].get_parameter_value().string_value, latching_qos
        )
        srv = node.create_service(
            GetEncoder, "/mirte/get_encoder_" + sensor["name"].get_parameter_value().string_value, self.get_data
        )
        self.speed_pub = node.create_publisher(
            Encoder, "/mirte/encoder_speed/" + sensor["name"].get_parameter_value().string_value, latching_qos
        )
        super().__init__(board, sensor, pub)
        self.ticks_per_wheel = 20
        if "ticks_per_wheel" in sensor:
            self.ticks_per_wheel = sensor["ticks_per_wheel"].get_parameter_value().integer_value
        self.max_freq = -1
        self.last_publish_value = Encoder()
        self.speed_count = 0

    def get_data(self, req, res):
        res.data = self.last_publish_value.value
        return res

    async def start(self):
        if board_mapping.get_mcu() == "pico":
            await self.board.set_pin_mode_encoder(
                self.pins["pin"], 0, self.publish_data, False
            )
        else:
            await self.board.set_pin_mode_encoder(
                self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data
            )
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.publish_speed_data)

    def publish_speed_data(self, event=None):
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = self.speed_count
        self.speed_count = 0
        self.speed_pub.publish(encoder)

    async def publish_data(self, data):
        self.speed_count = self.speed_count + 1
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = data[2]
        await self.publish(encoder)


class Servo:
    def __init__(self, board, servo_obj):
        self.board = board
        self.pins = get_pin_numbers(servo_obj)
        self.name = servo_obj["name"].get_parameter_value().string_value
        self.min_pulse = 544
        if "min_pulse" in servo_obj:
            self.min_pulse = servo_obj["min_pulse"].get_parameter_value().integer_value
        self.max_pulse = 2400
        if "max_pulse" in servo_obj:
            self.max_pulse = servo_obj["max_pulse"].get_parameter_value().integer_value

    async def stop(self):
        await self.board.detach_servo(self.pins["pin"])

    async def start(self):
        await self.board.set_pin_mode_servo(self.pins["pin"], self.min_pulse, self.max_pulse)
        server = node.create_service(
            SetServoAngle,
            "/mirte/set_" + self.name + "_servo_angle",
            self.set_servo_angle_service,
        )

    def set_servo_angle_service(self, req, res):
        asyncio.run(self.board.servo_write(self.pins["pin"], req.angle))
        res.status = True
        return res


class Motor:
    def __init__(self, board, motor_obj):
        self.board = board
        self.pins = get_pin_numbers(motor_obj)
        self.name = motor_obj["name"].get_parameter_value().string_value
        self.prev_motor_speed = 0
        self.initialized = False

    async def start(self):
        server = node.create_service(
            SetMotorSpeed,
            "/mirte/set_" + self.name + "_speed",
            self.set_motor_speed_service,
        )
        sub = node.create_subscription(
            Int32, "/mirte/motor_" + self.name + "_speed", self.callback, 10
        )

    def callback(self, data):
        asyncio.run(self.set_speed(data.data))

    def set_motor_speed_service(self, req, res):
        asyncio.run(self.set_speed(req.speed))
        res.status = True
        return res


class PPMotor(Motor):
    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where it creates a movement in the same direction.
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await set_pin_mode_analog_output(self.board, self.pins["p2"])
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
            if speed < 0:
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
                await set_pin_mode_analog_output(self.board, self.pins["p2"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.prev_motor_speed != speed:
            if speed == 0:
                await analog_write(self.board, self.pins["p2"], 0)
                await analog_write(self.board, self.pins["p1"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await analog_write(self.board, self.pins["p2"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await analog_write(self.board, self.pins["p1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p2"],
                    int(min(-speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            self.prev_motor_speed = speed


class DPMotor(Motor):
    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where it creates a movement in the same direction.
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await self.board.set_pin_mode_digital_output(self.pins["d1"])
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
            if speed < 0:
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
                await self.board.set_pin_mode_digital_output(self.pins["d1"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.prev_motor_speed != speed:
            if speed == 0:
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(self.board, self.pins["p1"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["d1"], 1)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(
                        board_mapping.get_max_pwm_value()
                        - min(abs(speed), 100)
                        / 100.0
                        * board_mapping.get_max_pwm_value()
                    ),
                )
            self.prev_motor_speed = speed


class DDPMotor(Motor):
    async def init_motors(self):
        if not self.initialized:
            await set_pin_mode_analog_output(self.board, self.pins["p1"])
            await self.board.set_pin_mode_digital_output(self.pins["d1"])
            await self.board.set_pin_mode_digital_output(self.pins["d2"])
            self.initialized = True

    async def set_speed(self, speed):
        # Make sure to set first set teh low pin. In this case the H-bridge
        # will never have two high pins.
        if self.prev_motor_speed != speed:
            await self.init_motors()
            if speed >= 0:
                await self.board.digital_write(self.pins["d1"], 0)
                await self.board.digital_write(self.pins["d2"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
                await self.board.digital_write(self.pins["d2"], 1)
            elif speed < 0:
                await self.board.digital_write(self.pins["d2"], 0)
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(
                        min(abs(speed), 100) / 100.0 * board_mapping.get_max_pwm_value()
                    ),
                )
                await self.board.digital_write(self.pins["d1"], 1)
            self.prev_motor_speed = speed


# Extended adafruit _SSD1306
class Oled(_SSD1306):
    def __init__(
        self,
        width,
        height,
        board,
        oled_obj,
        port,
        loop,
        addr=0x3C,
        external_vcc=False,
        reset=None,
    ):
        self.board = board
        self.oled_obj = oled_obj
        self.addr = addr
        self.temp = bytearray(2)
        self.i2c_port = port
        self.failed = False
        self.loop = loop
        self.init_awaits = []
        self.write_commands = []

        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        # self.buffer = bytearray(16)
        # self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        if board_mapping.get_mcu() == "pico":
            if "connector" in oled_obj:
                pins = board_mapping.connector_to_pins(oled_obj["connector"].get_parameter_value().string_value)
            else:
                pins = get_pin_numbers(oled_obj)
            pin_numbers = {}
            for item in pins:
                pin_numbers[item] = board_mapping.pin_name_to_pin_number(pins[item])
            self.i2c_port = board_mapping.get_I2C_port(pin_numbers["sda"])
            self.init_awaits.append(
                self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            )
        else:
            self.init_awaits.append(self.board.set_pin_mode_i2c(i2c_port=self.i2c_port))
        time.sleep(1)
        super().__init__(
            memoryview(self.buffer)[1:],
            width,
            height,
            external_vcc=external_vcc,
            reset=reset,
            page_addressing=False,
        )

    async def start(self):
        server = node.create_service(
            SetOLEDImage,
            "/mirte/set_" + self.oled_obj["name"].get_parameter_value().string_value + "_image",
            self.set_oled_image_service,
        )

        for ev in self.init_awaits:
            await ev
        for cmd in self.write_commands:
            out = await self.board.i2c_write(60, cmd, i2c_port=self.i2c_port)
            if (
                out == False
            ):  # pico returns true/false, arduino returns always none, only catch false
                print("write failed start", self.oled_obj["name"])
                self.failed = True
                return

    async def set_oled_image_service_async(self, req):
        if req.type == "text":
            text = req.value.replace("\\n", "\n")
            image = Image.new("1", (128, 64))
            draw = ImageDraw.Draw(image)
            split_text = text.splitlines()
            lines = []
            for i in split_text:
                lines.extend(textwrap.wrap(i, width=20))

            y_text = 1
            for line in lines:
                width, height = font.getsize(line)
                draw.text((1, y_text), line, font=font, fill=255)
                y_text += height
            self.image(image)
            await self.show_async()
        if req.type == "image":
            await self.show_png(
                "/usr/local/src/mirte/mirte-oled-images/images/" + req.value + ".png"
            )  # open color image

        if req.type == "animation":
            folder = (
                "/usr/local/src/mirte/mirte-oled-images/animations/" + req.value + "/"
            )
            number_of_images = len(
                [
                    name
                    for name in os.listdir(folder)
                    if os.path.isfile(os.path.join(folder, name))
                ]
            )
            for i in range(number_of_images):
                await self.show_png(folder + req.value + "_" + str(i) + ".png")

    def set_oled_image_service(self, req, res):
        if self.failed:
            print("oled writing failed")
            return SetOLEDImageResponse(False)

        try:
            # the ros service is started on a different thread than the asyncio loop
            # When using the normal loop.run_until_complete() function, both threads join in and the oled communication will get broken faster
            future = asyncio.run_coroutine_threadsafe(
                self.set_oled_image_service_async(req), self.loop
            )
            future.result()  # wait for it to be done
        except Exception as e:
            print(e)
        res.status = True
        return res

    def show(self):
        """Update the display"""
        xpos0 = 0
        xpos1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            xpos0 += 32
            xpos1 += 32
        if self.width == 72:
            # displays with width of 72 pixels are shifted by 28
            xpos0 += 28
            xpos1 += 28
        self.write_cmd(0x21)  # SET_COL_ADDR)
        self.write_cmd(xpos0)
        self.write_cmd(xpos1)
        self.write_cmd(0x22)  # SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.write_commands.append([0x80, cmd])

    async def write_cmd_async(self, cmd):
        if self.failed:
            return
        self.temp[0] = 0x80
        self.temp[1] = cmd
        out = await self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port)
        if out == False:
            print("failed write oled 2")
            self.failed = True

    async def show_async(self):
        """Update the display"""
        # TODO: only update pixels that are changed
        xpos0 = 0
        xpos1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            xpos0 += 32
            xpos1 += 32
        if self.width == 72:
            # displays with width of 72 pixels are shifted by 28
            xpos0 += 28
            xpos1 += 28

        try:
            cmds = [
                self.write_cmd_async(0x21),  # SET_COL_ADDR)
                self.write_cmd_async(xpos0),
                self.write_cmd_async(xpos1),
                self.write_cmd_async(0x22),  # SET_PAGE_ADDR)
                self.write_cmd_async(0),
                self.write_cmd_async(self.pages - 1),
                *self.write_framebuf_async(),
            ]
            await asyncio.gather(*cmds)
        except Exception as e:
            print(e)

    def write_framebuf_async(self):
        if self.failed:
            return

        async def task(self, i):
            buf = self.buffer[i * 16 : (i + 1) * 16 + 1]
            buf[0] = 0x40
            out = await self.board.i2c_write(60, buf, i2c_port=self.i2c_port)
            if out == False:
                print("failed wrcmd")
                self.failed = True

        tasks = []
        for i in range(64):
            tasks.append(task(self, i))
        return tasks

    def write_framebuf(self):
        for i in range(64):
            buf = self.buffer[i * 16 : (i + 1) * 16 + 1]
            buf[0] = 0x40
            self.write_commands.append(buf)

    async def show_png(self, file):
        image_file = Image.open(file)  # open color image
        image_file = image_file.convert("1", dither=Image.NONE)
        self.image(image_file)
        await self.show_async()


async def handle_set_led_value(req):
    led = rospy.get_param("/mirte/led")
    await analog_write(
        board,
        get_pin_numbers(led)["pin"],
        int(min(req.value, 100) / 100.0 * board_mapping.get_max_pwm_value()),
    )
    return SetLEDValueResponse(True)


# TODO: This needs a full refactor. Probably needs its own class
# with a member storing all settings of the pins (analog/digital)
# and whether or not a callback needs to be called.
# It pwill prbably only need one callback function anyway, pushing
# the values into the member variable.

pin_values = {}


# TODO: and this one probably needs to keep track of
# time as well, making sure that I can not call
# this one more often than another pin.
async def data_callback(data):
    global pin_values
    pin_number = data[1]
    if data[0] == 3:
        pin_number += board_mapping.get_analog_offset()
    pin_values[pin_number] = data[2]


def handle_get_pin_value(req):
    global pin_values
    # Map pin to the pin map if it is in there, or to
    # an int if raw pin number
    try:
        pin = board_mapping.pin_name_to_pin_number(req.pin)
    except:
        pin = int(req.pin)

    if not pin in pin_values:
        if req.type == "analog":
            asyncio.run(
                board.set_pin_mode_analog_input(
                    pin - board_mapping.get_analog_offset(), callback=data_callback
                )
            )
        if req.type == "digital":
            asyncio.run(board.set_pin_mode_digital_input(pin, callback=data_callback))

    # timeout after 5s, don't keep waiting on something that will never happen.
    start_time = time.time()
    while not pin in pin_values and time.time() - start_time < 5.0:
        time.sleep(0.001)
    if pin in pin_values:
        value = pin_values[pin]
    else:
        value = -1  # device did not report back, so return error value.

    value = pin_values[pin]
    return GetPinValueResponse(value)


# TODO: check on existing pin configuration?
def handle_set_pin_value(req):
    # Map pin to the pin map if it is in there, or to
    # an int if raw pin number
    try:
        pin = board_mapping.pin_name_to_pin_number(req.pin)
    except:
        pin = int(req.pin)

    if req.type == "analog":
        # This should be a PWM capable pin. Therefore we do not need to
        # account for the board_mapping.analog_offset. We do need to account for the
        # max pwm_value though.
        capped_value = min(req.value, board_mapping.get_max_pwm_value())
        asyncio.run(set_pin_mode_analog_output(board, pin))
        asyncio.run(asyncio.sleep(0.001))
        asyncio.run(analog_write(board, pin, capped_value))
    if req.type == "digital":
        asyncio.run(board.set_pin_mode_digital_output(pin))
        asyncio.run(asyncio.sleep(0.001))
        asyncio.run(board.digital_write(pin, req.value))
    return SetPinValueResponse(True)


# Initialize the actuators. Each actuator will become a service
# which can be called.
def actuators(loop, board, device):
    servers = []

    if ("oled" in params):
        oleds = params["oled"]
        oleds = {k: v for k, v in oleds.items() if v["device"].get_parameter_value().string_value == device}
        oled_id = 0
        for oled in oleds:
            oled_obj = Oled(
                128, 64, board, oleds[oled], port=oled_id, loop=loop
            )  # get_pin_numbers(oleds[oled]))
            oled_id = oled_id + 1
            servers.append(loop.create_task(oled_obj.start()))

    # TODO: support multiple leds
    if ("led" in params):
        led = params["led"]
        loop.run_until_complete(
            set_pin_mode_analog_output(board, get_pin_numbers(led)["pin"])
        )
        server = node.create_service(
            SetLEDValue, "/mirte/set_led_value", handle_set_led_value
        )
        servers.append(loop.create_task(server.start()))

    if ("motor" in params):
        motors = params["motor"]
        motors = {k: v for k, v in motors.items() if v["device"].get_parameter_value().string_value == device}
        for motor in motors:
            motor_obj = {}
            motor_type = motors[motor]["type"].get_parameter_value().string_value
            if motor_type == "ddp":
                motor_obj = DDPMotor(board, motors[motor])
            elif motor_type == "dp":
                motor_obj = DPMotor(board, motors[motor])
            elif motor_type == "pp":
                motor_obj = PPMotor(board, motors[motor])
            else:
                node.get_logger().info(f'Unsupported motor interface (ddp, dp, or pp)', once=True)
            servers.append(loop.create_task(motor_obj.start()))

    if ("servo" in params):
        servos = params["servo"]
        servos = {k: v for k, v in servos.items() if v["device"].get_parameter_value().string_value == device}
        for servo in servos:
            servo = Servo(board, servos[servo])
            servers.append(loop.create_task(servo.start()))

    # Set a raw pin value
    server = node.create_service(SetPinValue, "/mirte/set_pin_value", handle_set_pin_value)

    return servers


# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
    tasks = []
    max_freq = 30
    #if params["device"]["mirte"]["max_frequency"]:
    #    max_freq = params["device"]["mirte"]["max_frequency"].get_parameter_value().integer_value

    # For now, we need to set the analog scan interval to teh max_freq. When we set
    # this to 0, we do get the updates from telemetrix as fast as possible. In that
    # case the aiorospy creates a latency for the analog sensors (data will be
    # updated with a delay). This also happens when you try to implement this with
    # nest_asyncio icw rospy services.
    # Maybe there is a better solution for this, to make sure that we get the
    # data here asap.
    if board_mapping.get_mcu() == "pico":
        if max_freq <= 1:
            tasks.append(loop.create_task(board.set_scan_delay(1)))
        else:
            try:
                tasks.append(
                    loop.create_task(board.set_scan_delay(int(1000.0 / max_freq)))
                )
            except:
                print("failed scan delay")
                pass
    else:
        if max_freq <= 0:
            tasks.append(loop.create_task(board.set_analog_scan_interval(0)))
        else:
            tasks.append(
                loop.create_task(board.set_analog_scan_interval(int(1000.0 / max_freq)))
            )

    # initialze distance sensors
    if ("distance" in params):
        distance_sensors = params["distance"]
        distance_sensors = {
            k: v for k, v in distance_sensors.items() if v["device"].get_parameter_value().string_value == device
        }
        for sensor in distance_sensors:
            distance_sensors[sensor]["max_frequency"] = max_freq
            monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))

    # Initialize intensity sensors
    if ("intensity" in params):
        intensity_sensors = params["intensity"]
        intensity_sensors = {
            k: v for k, v in intensity_sensors.items() if v["device"].get_parameter_value().string_value == device
        }
        for sensor in intensity_sensors:
            intensity_sensors[sensor]["max_frequency"] = max_freq
            if "analog" in get_pin_numbers(intensity_sensors[sensor]):
                monitor = AnalogIntensitySensorMonitor(board, intensity_sensors[sensor])
                tasks.append(loop.create_task(monitor.start()))
            if "digital" in get_pin_numbers(intensity_sensors[sensor]):
                monitor = DigitalIntensitySensorMonitor(
                    board, intensity_sensors[sensor]
                )
                tasks.append(loop.create_task(monitor.start()))

    # Initialize keypad sensors
    if ("keypad" in params):
        keypad_sensors = params["keypad"]
        keypad_sensors = {
            k: v for k, v in keypad_sensors.items() if v["device"].get_parameter_value().string_value == device
        }
        for sensor in keypad_sensors:
            keypad_sensors[sensor]["max_frequency"] = max_freq
            monitor = KeypadMonitor(board, keypad_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))

    # Initialize encoder sensors
    if ("encoder" in params):
        encoder_sensors = params("encoder")
        encoder_sensors = {
            k: v for k, v in encoder_sensors.items() if v["device"].get_parameter_value().string_value == device == device
        }
        for sensor in encoder_sensors:
            monitor = EncoderSensorMonitor(board, encoder_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
            # encoder sensors do not need a max_frequency. They are interrupts on
            # on the mcu side.

    # Get a raw pin value
    # TODO: this still needs to be tested. We are waiting on an implementation of ananlog_read()
    # on the telemetrix side
    node.create_service(GetPinValue, "/mirte/get_pin_value", handle_get_pin_value)

    return tasks


# Shutdown procedure
closing = False


def shutdown(loop, board):
    global closing

    # We need to check if this closing is not already
    # running by an escalated signal.
    if not closing:
        closing = True
        loop.run_until_complete(board.shutdown())
        print("Telemetrix shutdown nicely")
        # Stop the asyncio loop
        loop.stop()
        print("Loop shutdown nicely")
        node.destroy_node()
#        rclpy.shutdown()
        time.sleep(1)
        exit(0)


def test():
   print("halloooo")

async def spinonce():
    rclpy.spin_once(node, timeout_sec=0)


async def rosspin():
   while True:
      print("hallo")
      await spin_once()
      await asyncio.sleep(0.0)







def main(arg=None):
    loop = asyncio.new_event_loop()


    # Initialize the telemetrix board
    if board_mapping.get_mcu() == "pico":
        board = tmx_pico_aio.TmxPicoAio(
            allow_i2c_errors=True,
            loop=loop,
            autostart=False,
#            com_port="/dev/ttyUSB0"
        )
        loop.run_until_complete(board.start_aio())
    else:
        board = telemetrix_aio.TelemetrixAIO()

    # needed for the loop.run_forevr() 
    time.sleep(2)

    # Catch signals to exit properly
    # We need to do it this way instead of usgin the try/catch
    # as in the telemetrix examples
#    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
#    for s in signals:
#        l = lambda loop=loop, board=board: asyncio.ensure_future(shutdown(loop, board))
#        loop.add_signal_handler(s, l)

#    loop.add_signal_handler(signal.SIGINT, asyncio.ensure_future(shutdown(loop, board)))

    # Initialize the ROS node as anonymous since there
    # should only be one instnace running.
    #rospy.init_node("mirte_telemetrix", anonymous=False)

    # Escalate siging to this process in order to shutdown nicely
    # This is needed when only this process is killed (eg. rosnode kill)
    # This cannot be done by calling shutdown() because that is
    # a different thread without asyncio loop.
    #l = lambda pid=os.getpid(), sig=signal.SIGINT: os.kill(pid, sig)
    #rospy.on_shutdown(l)

    # Start all tasks for sensors and actuators
    device = "mirte"
    sensor_tasks = sensors(loop, board, device)
    actuator_tasks = actuators(loop, board, device)
    all_tasks = sensor_tasks + actuator_tasks
    for task in all_tasks:
        loop.run_until_complete(task)


    # Is equivalent to rclpy.spin(node) in a sense that this
    # will just keep the node running only in a asyncio
    # way.
#    loop.run_forever()

    try:
#        task_ros2 = asyncio.create_task(rosspin())
        # task_mavsdk = asyncio.create_task(run(minimal_publisher))
        #asyncio.run(task_ros2)
#        await asyncio.gather(task_ros2)
        #loop.run_until_complete(rosspin())
        loop.run_forever()   # this will make teh sensors work
#        rclpy.spin(node)      # this will make the actuators work

    except:
        pass
    finally:
        node.on_shutdown()

    shutdown(loop, board)

def start():
   main()
#   asyncio.run(main())

if __name__ == "__main__":
   main()
