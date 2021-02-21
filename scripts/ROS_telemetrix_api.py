#!/usr/bin/env python3.8
import asyncio
import os, os.path
import sys
import time
import math
import rospy
import signal
import aiorospy
import io
from telemetrix_aio import telemetrix_aio

# Import ROS message types
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import *

# Import ROS services
from zoef_msgs.srv import *

from bitstring import BitArray
import textwrap

from PIL import Image, ImageDraw, ImageFont

#font = ImageFont.truetype("/usr/share/fonts/truetype/terminus.ttf", 12)
font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 12)

from adafruit_ssd1306 import _SSD1306
from concurrent.futures import ThreadPoolExecutor
executor = ThreadPoolExecutor(10)


# Map to convert from STM32 to pin numbers
# TODO: maybe convert this into JSON files and load it from there
# TODO: maybe just make this an list and get the index of the list
# And/or put this in the param server so teh web interface can also use this
stm32_map = {
 "B9" : 0,
 "B8" : 1,
 "B7" : 2,
 "B6" : 3,
 "B5" : 4,
 "B4" : 5,
 "B3" : 6,
 "A15": 7,
 "A12": 8,
 "A11": 9,
 "A10": 10,
 "A9" : 11,
 "A8" : 12,
 "B15": 13,
 "B14": 14,
 "B13": 15,
 "B12": 16,
 "C13": 17,    # LED
 "C14": 18,
 "C15": 19,
 "A0" : 20,
 "A1" : 21,
 "A2" : 22,
 "A3" : 23,
 "A4" : 24,
 "A5" : 25,
 "A6" : 26,
 "A7" : 27,
 "B0" : 28,
 "B1" : 29,
 "B10": 30,
 "B11": 31
}
stm32_max_pwm_value = 65535
stm32_analog_offset = 20

nano_map = {
"RX0" : 0,
"TX1" : 1,
"D2"  : 2,
"D3"  : 3,
"D4"  : 4,
"D5"  : 5,
"D6"  : 6,
"D7"  : 7,
"D8"  : 8,
"D9"  : 9,
"D10" : 10,
"D11" : 11,
"D12" : 12,
"D13" : 13,
"A0"  : 14,
"A1"  : 15,
"A2"  : 16,
"A3"  : 17,
"A4"  : 18,
"A5"  : 19,
"A6"  : 20,
"A7"  : 21
}
nano_max_pwm_value = 255
nano_analog_offset = 14

# Map to convert from Zoef PCB to STM32 pins numbers
# This should be the same as printed on the PCB
zoef_pcb_map = {
 "IR1"  : {"digital": "B1" , "analog": "A0" },
 "IR2"  : {"digital": "B0" , "analog": "A1" },
 "SRF1" : {"trigger": "A9" , "echo"  : "B8" },
 "SRF2" : {"trigger": "A10", "echo"  : "B9" },
 "I2C1" : {"scl"    : "B6" , "sda"   : "B7" },
 "I2C2" : {"scl"    : "B10", "sda"   : "B11"},
 "ENCA" : {"pin"    : "B12"},
 "ENCB" : {"pin"    : "B13"},
 "KEY"  : {"pin"    : "A4" },
 "SERVO": {"pin"    : "B5" },
 "LED"  : {"pin"    : "B4" },
 "MA"   : {"1a"     : "A8" , "1b"    : "B3" },
 "MB"   : {"1a"     : "B14", "1b"    : "B15"}
}

# Determine the analog offset, based on the mcu
# TODO: this shuold be refactored together with the
# mapping above
devices = rospy.get_param('/zoef/device')
analog_offset = 0
max_pwm_value = 0
pin_map = {}
if devices["zoef"]["type"] == "zoef_pcb" or devices["zoef"]["mcu"] == "stm32":
   analog_offset = stm32_analog_offset
   max_pwm_value = stm32_max_pwm_value
   pin_map = stm32_map
elif devices["zoef"]["mcu"] == "nano":
   analog_offset = nano_analog_offset
   max_pwm_value = nano_max_pwm_value
   pin_map = nano_map
else:
   max_pwm_value = 255 # TODO: also make this a setting

def get_pin_numbers(component):
   devices = rospy.get_param('/zoef/device')
   device = devices[component["device"]]
   pins = {}
   if device["type"] == "zoef_pcb":
      mcu = "stm32"
      pins = zoef_pcb_map[component["connector"]]
   elif device["type"] == "breadboard":
      mcu = device["mcu"]
      pins = component["pins"]

   # convert pin naming to numbers
   pin_numbers = {}
   for item in pins:
      if mcu == "stm32":
         pin_numbers[item] = stm32_map[pins[item]]
      elif mcu == "nano":
         pin_numbers[item] = nano_map[pins[item]]
      else:
         pin_numbers[item] = pins[item]
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
        rospy.loginfo("Sensor initialized on topic %s (max_freq: %d, differential: %d)", self.publisher.name, self.max_freq, self.differential)

    def get_header(self):
       header = Header()
       header.stamp = rospy.Time.now()
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
             self.last_publish_time += (1000.0 / self.max_freq) # Note: this should not be set to now_millis. This is due to Nyquist.

class KeypadMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher('/zoef/keypad/' + sensor["name"], Keypad, queue_size=1)
        srv = rospy.Service('/zoef/get_keypad_' + sensor["name"], GetKeypad, self.get_data)
        super().__init__(board, sensor, pub)
        self.last_debounce_time = 0
        self.last_key = ""
        self.last_debounced_key = ""
        self.pressed_publisher = rospy.Publisher('/zoef/keypad/' + sensor["name"] + '_pressed', Keypad, queue_size=1)
        self.last_publish_value = Keypad()

    def get_data(self, req):
        return GetKeypadResponse(self.last_publish_value.key)

    async def start(self):
        await self.board.set_pin_mode_analog_input(self.pins["pin"] - analog_offset, self.differential, self.publish_data)

    async def publish_data(self, data):
       # Determine the key that is pressed
       key = ""
       if data[2] < 70:
          key = "left"
       elif data[2] < 230:
          key = "up"
       elif data[2] < 410:
          key = "down"
       elif data[2] < 620:
          key = "right"
       elif data[2] < 880:
          key = "enter"

       # Do some debouncing
       if self.last_key is not key:
          self.last_debounce_time = data[3]

       debounced_key = ""
       if data[3] - self.last_debounce_time > .1:
          debounced_key = key

       # Publish the last debounced key
       keypad = Keypad()
       keypad.header = self.get_header()
       keypad.key = debounced_key
       await self.publish(keypad)

       # check if we need to send a pressed message
       if (self.last_debounced_key != "") and (self.last_debounced_key is not debounced_key):
          pressed = Keypad()
          pressed.header = self.get_header()
          pressed.key = self.last_debounced_key
          self.pressed_publisher.publish(pressed)

       self.last_key = key
       self.last_debounced_key = debounced_key

class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher('/zoef/distance/' + sensor["name"], Range, queue_size=1, latch=True)
        srv = rospy.Service('/zoef/get_distance_' + sensor["name"], GetDistance, self.get_data)
        super().__init__(board, sensor, pub)
        self.last_publish_value = Range()

    def get_data(self, req):
        return GetDistanceResponse(self.last_publish_value.range)

    async def start(self):
        print(self.pins)
        await self.board.set_pin_mode_sonar(self.pins["trigger"], self.pins["echo"], self.publish_data)

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
        range.range = data[2]
        await self.publish(range)

class DigitalIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher('/zoef/intensity/' + sensor["name"] + "_digital", IntensityDigital, queue_size=1, latch=True)
        srv = rospy.Service('/zoef/get_intensity_' + sensor["name"] + "_digital", GetIntensityDigital, self.get_data)
        super().__init__(board, sensor, pub)
        self.last_publish_value = IntensityDigital()

    def get_data(self, req):
        return GetIntensityDigitalResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_digital_input(self.pins["digital"], callback=self.publish_data)

    async def publish_data(self, data):
        intensity = IntensityDigital()
        intensity.header = self.get_header()
        intensity.value = bool(data[2])
        await self.publish(intensity)

class AnalogIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher('/zoef/intensity/' + sensor["name"], Intensity, queue_size=1)
        srv = rospy.Service('/zoef/get_intensity_' + sensor["name"], GetIntensity, self.get_data)
        super().__init__(board, sensor, pub)
        self.last_publish_value = Intensity()

    def get_data(self, req):
        return GetIntensityResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_analog_input(self.pins["analog"] - analog_offset, differential=self.differential, callback=self.publish_data)

    async def publish_data(self, data):
        intensity = Intensity()
        intensity.header = self.get_header()
        intensity.value = data[2]
        await self.publish(intensity)

class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher('/zoef/encoder/' + sensor["name"], Encoder, queue_size=1, latch=True)
        srv = rospy.Service('/zoef/get_encoder_' + sensor["name"], GetEncoder, self.get_data)
        self.speed_pub = rospy.Publisher('/zoef/encoder_speed/' + sensor["name"], Encoder, queue_size=1, latch=True)
        super().__init__(board, sensor, pub)
        self.ticks_per_wheel = sensor["ticks_per_wheel"]
        self.max_freq = -1
        self.last_publish_value = Encoder()
        self.speed_count = 0

    def get_data(self, req):
        return GetEncoderResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_encoder(self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data)
        rospy.Timer(rospy.Duration(1.0/10.0), self.publish_speed_data)

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

class PWMMotor():
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_speed = 0
        self.loop = asyncio.get_event_loop()
        self.initialized = False

    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where is creates a movement in teh same direction.
    async def init_motors(self, speed):
        if not self.initialized:
          if (speed > 0):
            await self.board.set_pin_mode_analog_output(self.pins["1a"])
            await self.board.set_pin_mode_analog_output(self.pins["1b"])
          if (speed < 0):
            await self.board.set_pin_mode_analog_output(self.pins["1b"])
            await self.board.set_pin_mode_analog_output(self.pins["1a"])
          self.initialized = True

    async def set_speed(self, speed):
        if (self.prev_motor_speed != speed):
          if (speed == 0):
            await self.board.analog_write(self.pins["1a"], 0)
            await self.board.analog_write(self.pins["1b"], 0)
          elif (speed > 0):
            await self.init_motors(speed)
            await self.board.analog_write(self.pins["1a"], 0)
            await self.board.analog_write(self.pins["1b"], int(min(speed, 100) / 100.0 * max_pwm_value))
          elif (speed < 0):
            await self.init_motors(speed)
            await self.board.analog_write(self.pins["1b"], 0)
            await self.board.analog_write(self.pins["1a"], int(min(abs(speed), 100) / 100.0 * max_pwm_value))
          self.prev_motor_speed = speed

class L298NMotor():
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_speed = 0
        self.loop = asyncio.get_event_loop()
        self.initialized = False

    async def init_motors(self):
        if not self.initialized:
           await self.board.set_pin_mode_analog_output(self.pins["en"])
           await self.board.set_pin_mode_digital_output(self.pins["in1"])
           await self.board.set_pin_mode_digital_output(self.pins["in2"])
           self.initialized = True

    async def set_speed(self, speed):
        # Make sure to set first set teh low pin. In this case the H-bridge
        # will never have two high pins.
        if (self.prev_motor_speed != speed):
          self.init_motors()
          if (speed >= 0):
            await self.board.digital_write(self.pins["in1"], 0)
            await self.board.digital_write(self.pins["in2"], 1)
            await self.board.analog_write(self.pins["en"], int(min(speed, 100) / 100.0 * max_pwm_value))
          elif (speed < 0):
            await self.board.digital_write(self.pins["in2"], 0)
            await self.board.digital_write(self.pins["in1"], 1)
            await self.board.analog_write(self.pins["en"], int(min(abs(speed), 100) / 100.0 * max_pwm_value))
          self.prev_motor_speed = speed

# Extended adafruit _SSD1306
class Oled(_SSD1306):
    def __init__(
        self, width, height, board, loop, port, addr=0x3C, external_vcc=False, reset=None
    ):
        self.board = board
        self.addr = addr
        self.temp = bytearray(2)
        self.i2c_port = port
        self.loop = asyncio.get_event_loop()
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        #self.buffer = bytearray(16)
        #self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        self.loop.run_until_complete(self.board.set_pin_mode_i2c(i2c_port=self.i2c_port))
        super().__init__(
            memoryview(self.buffer)[1:],
            width,
            height,
            external_vcc=external_vcc,
            reset=reset,
        )

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
        self.write_cmd(0x21) #SET_COL_ADDR)
        self.write_cmd(xpos0)
        self.write_cmd(xpos1)
        self.write_cmd(0x22) #SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def write_cmd(self, cmd):
        """Send a command to the SPI device"""
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        # this can be called from within a loop (sytarted by write_framebuff) or out of itseflt, without a running loop
        #loop = self.loop
        #if loop.is_running():
        #   loop = asyncio.get_running_loop()
        #await self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port)
        #await self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port)
        self.loop.run_until_complete(self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port)) # await not possible since it is called by parents

    def write_framebuf(self):
        """Blast out the frame buffer using a single I2C transaction to support
        hardware I2C interfaces."""
        #loop = asyncio.get_running_loop()
        #self.loop.run_until_complete(self.board.i2c_write(60, [0x40]))
        for i in range(64):  #TODO: can we have higher i2c buffer (limited by firmata 64 bits and wire 32 bits, so actually 16 bits since we need 1 bit)
            buf = self.buffer[i*16:(i+1)*16+1]
            buf[0] = 0x40
            #await self.board.i2c_write(60, buf, i2c_port=self.i2c_port)
            self.loop.run_until_complete(self.board.i2c_write(60, buf, i2c_port=self.i2c_port)) # can not be awaited, since it is called

    def show_png(self, file):
      image_file = Image.open(file) # open color image
      image_file = image_file.convert('1', dither=Image.NONE)
      self.image(image_file)
      self.show()


async def set_motor_speed_service(req, motor):
    await motor.set_speed(req.speed)
    return SetMotorSpeedResponse(True)

async def handle_set_led_value(req):
    led = rospy.get_param("/zoef/led")
    await board.analog_write(get_pin_numbers(led)["pin"], int(min(req.value, 100) / 100.0 * max_pwm_value))
    return SetLEDValueResponse(True)

async def handle_set_servo_angle(req, servo_pin):
    await board.servo_write(servo_pin, req.angle)
    return SetServoAngleResponse(True)


import nest_asyncio
nest_asyncio.apply()

# TODO: This needs a full refactor. Probably needs its own class
# with a member storing all settings of the pins (analog/digital)
# and whether or not a callback needs to be called.
# It pwill prbably only need one callback function anyway, pushing
# the values into the member variable.
import nest_asyncio
nest_asyncio.apply()

pin_values = {}

# TODO: and this one probably needs to keep track of
# time as well, making sure that I can not call
# this one more often than another pin.
async def data_callback(data):
    global pin_values
    print(data)
    pin_number = data[1]
    if data[0] == 3:
        pin_number += analog_offset
    pin_values[pin_number] = data[2]

def handle_get_pin_value(req):
   global pin_values
   if not req.pin in pin_values:
      if req.type == "analog":
         print("heieer analog" )
         loop.run_until_complete(board.set_pin_mode_analog_input(req.pin - analog_offset, callback=data_callback))
      if req.type == "digital":
         print("digitlaal" )
         loop.run_until_complete(board.set_pin_mode_digital_input(req.pin, callback=data_callback))

   print(pin_values)
   while not req.pin in pin_values:
      time.sleep(.00001)
   value = pin_values[req.pin]
   return GetPinValueResponse(value)

#TODO: make faster with asyncio
def set_oled_image_service(req, oled):
    if req.type == "text":
      text = req.value.replace('\\n', '\n')
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
      oled.image(image)
      oled.show()
      return SetOLEDImageResponse(True)

    if req.type == "image":
      oled.show_png("/usr/local/src/zoef/zoef_oled_images/images/" + req.value + ".png") # open color image
      return SetOLEDImageResponse(True)

    if req.type == "animation":
      folder = "/usr/local/src/zoef/zoef_oled_images/animations/" +  req.value + "/"
      number_of_images = len([name for name in os.listdir(folder) if os.path.isfile(os.path.join(folder, name))])
      for i in range(number_of_images):
         oled.show_png(folder + req.value + "_" + str(i) + ".png")
      return SetOLEDImageResponse(True)

# TODO: check on existing pin configuration?
async def handle_set_pin_value(req):
  # Map pin to the pin map if it is in there, or to
  # an int if raw pin number
  if req.pin in pin_map:
     pin = pin_map[req.pin]
  else:
     pin = int(req.pin)

  if req.type == "analog":
     # This should be a PWM capable pin. Therefore we do not need to
     # account for the analog_offset. We do need to account for the
     # max pwm_value though.
     capped_value = min(req.value, max_pwm_value)
     await board.set_pin_mode_analog_output(pin)
     await board.analog_write(pin, capped_value)
  if req.type == "digital":
     await board.set_pin_mode_digital_output(pin)
     await board.digital_write(pin, req.value)
  return SetPinValueResponse(True)


# Shutdown procedure
async def shutdown(signal, loop, board):
    # Shutdown teh telemtrix board
    await board.shutdown()

    # Stop the asyncio loop
    loop.stop()

    # Shutdown all tasks
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)

    # Exit
    exit(0)

# Initialize the actuators. Each actuator will become a service
# which can be called.
def actuators(loop, board, device):
    servers = []

    if rospy.has_param("/zoef/oled"):
       oleds = rospy.get_param("/zoef/oled")
       oleds = {k: v for k, v in oleds.items() if v['device'] == device}
       oled_id = 0
       for oled in oleds:
          oled_obj = Oled(128, 64, board, loop, port=oled_id) #get_pin_numbers(oleds[oled]))
          l = lambda req,o=oled_obj: set_oled_image_service(req, o)
          syncServer = rospy.Service("/zoef/set_" + oled + "_image", SetOLEDImage, l)
          oled_id = oled_id + 1
       #servers.append(loop.create_task(server.start()))

    # TODO: support multiple leds
    if rospy.has_param("/zoef/led"):
       led = rospy.get_param("/zoef/led")
       loop.run_until_complete(board.set_pin_mode_analog_output(get_pin_numbers(led)["pin"]))
       server = aiorospy.AsyncService('/zoef/set_led_value', SetLEDValue, handle_set_led_value)
       servers.append(loop.create_task(server.start()))

    if rospy.has_param("/zoef/servo"):
        servos = rospy.get_param("/zoef/servo")
        servos = {k: v for k, v in servos.items() if v['device'] == device}
        for servo in servos:
           pin = get_pin_numbers(servos[servo])["pin"]
           loop.run_until_complete(board.set_pin_mode_servo(pin))
           l = lambda req,p=pin: handle_set_servo_angle(req, p)
           server = aiorospy.AsyncService('/zoef/set_' + servo + '_servo_angle', SetServoAngle, l)
           servers.append(loop.create_task(server.start()))

    if rospy.has_param("/zoef/motor"):
       motors = rospy.get_param("/zoef/motor")
       motors = {k: v for k, v in motors.items() if v['device'] == device}
       for motor in motors:
          motor_obj = {}
          if motors[motor]["type"] == "l298n":
             motor_obj = L298NMotor(board, get_pin_numbers(motors[motor]))
          else:
             motor_obj = PWMMotor(board, get_pin_numbers(motors[motor]))
          l = lambda req,m=motor_obj: set_motor_speed_service(req, m)
          server = aiorospy.AsyncService("/zoef/set_" + motor + "_speed", SetMotorSpeed, l)
          servers.append(loop.create_task(server.start()))

    # Set a raw pin value
    server = aiorospy.AsyncService('/zoef/set_pin_value', SetPinValue, handle_set_pin_value)
    servers.append(loop.create_task(server.start()))

    return servers


# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
   tasks = []
   max_freq = -1

   # initialze distance sensors
   if rospy.has_param("/zoef/distance"):
      distance_sensors = rospy.get_param("/zoef/distance")
      distance_sensors = {k: v for k, v in distance_sensors.items() if v['device'] == device}
      for sensor in distance_sensors:
         distance_publisher = rospy.Publisher('/zoef/' + sensor, Range, queue_size=1, latch=True)
         monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
         tasks.append(loop.create_task(monitor.start()))
         if "max_frequency" in distance_sensors[sensor] and distance_sensors[sensor]["max_frequency"] >= max_freq:
            max_freq = distance_sensors[sensor]["max_frequency"]
         else:
            max_freq = 10

   # Initialize intensity sensors
   if rospy.has_param("/zoef/intensity"):
      intensity_sensors = rospy.get_param("/zoef/intensity")
      intensity_sensors = {k: v for k, v in intensity_sensors.items() if v['device'] == device}
      for sensor in intensity_sensors:
         if "analog" in get_pin_numbers(intensity_sensors[sensor]):
            monitor = AnalogIntensitySensorMonitor(board, intensity_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
         if "digital" in get_pin_numbers(intensity_sensors[sensor]):
            monitor = DigitalIntensitySensorMonitor(board, intensity_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
         if "max_frequency" in intensity_sensors[sensor] and intensity_sensors[sensor]["max_frequency"] >= max_freq:
            max_freq = intensity_sensors[sensor]["max_frequency"]
         else:
            max_freq = 10

   # Initialize keypad sensors
   if rospy.has_param("/zoef/keypad"):
      keypad_sensors = rospy.get_param("/zoef/keypad")
      keypad_sensors = {k: v for k, v in keypad_sensors.items() if v['device'] == device}
      for sensor in keypad_sensors:
         monitor = KeypadMonitor(board, keypad_sensors[sensor])
         tasks.append(loop.create_task(monitor.start()))
         if "max_frequency" in keypad_sensors[sensor] and keypad_sensors[sensor]["max_frequency"] >= max_freq:
            max_freq = keypad_sensors[sensor]["max_frequency"]
         else:
            max_freq = 10

   # Initialize encoder sensors
   if rospy.has_param("/zoef/encoder"):
      encoder_sensors = rospy.get_param("/zoef/encoder")
      encoder_sensors = {k: v for k, v in encoder_sensors.items() if v['device'] == device}
      for sensor in encoder_sensors:
         monitor = EncoderSensorMonitor(board, encoder_sensors[sensor])
         tasks.append(loop.create_task(monitor.start()))
         # encoder sensors do not need a max_frequency. They are interrupts on
         # on the mcu side.

   # Get a raw pin value
   # TODO: this still needs to be tested. We are waiting on an implementation of ananlog_read()
   # on the telemetrix side
   rospy.Service('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
   #server = aiorospy.AsyncService('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
   #tasks.append(loop.create_task(server.start()))

   # For now, we need to set the analog scan interval to teh max_freq. When we set
   # this to 0, we do get the updates from telemetrix as fast as possible. In that
   # case the aiorospy creates a latency for the analog sensors (data will be
   # updated with a delay). This also happens when you try to implement this with
   # nest_asyncio icw rospy services.
   # Maybe there is a better solution for this, to make sure that we get the
   # data here asap.
   if max_freq <= 0:
      loop.run_until_complete(board.set_analog_scan_interval(0))
   else:
      loop.run_until_complete(board.set_analog_scan_interval(int(1000.0/max_freq)))

   return tasks


def send_sigint():
   os.kill(os.getpid(), signal.SIGINT)

if __name__ == '__main__':
   loop = asyncio.get_event_loop()

   # Catch signals to exit properly
   # We need to do it this way instead of usgin the try/catch
   # as in the telemetrix examples
   signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
   for s in signals:
      loop.add_signal_handler(s, lambda: asyncio.ensure_future(shutdown(s, loop, board)))

   # Initialize the telemetrix board
   board = telemetrix_aio.TelemetrixAIO(loop=loop)

   # Initialize the ROS node as anonymous since there
   # should only be one instnace running.
   rospy.init_node('zoef_telemetrix', anonymous=False, disable_signals=False)
   rospy.on_shutdown(send_sigint)

   # Start all tasks for sensors and actuators
   device = 'zoef'
   sensor_tasks = sensors(loop, board, device)
   actuator_tasks = actuators(loop, board, device)
   all_tasks = sensor_tasks + actuator_tasks
   for task in all_tasks:
       loop.run_until_complete(task)

   # Should not be: loop.run_forever() or rospy.spin()
   # Loop forever and give async some time to process
   # The sleep time should be lower than 1/max_freq
   # Since telemetrix updates at a max of 1ms, 0.00001
   # should be enough.
   while True:
      loop.run_until_complete(asyncio.sleep(0.00001))
