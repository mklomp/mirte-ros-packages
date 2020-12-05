#!/usr/bin/env python3.8
import asyncio
import sys
import time
import math
import rospy
import signal
import aiorospy
from telemetrix_aio import telemetrix_aio
from concurrent.futures import ProcessPoolExecutor

from std_msgs.msg import Int32, Empty, String, Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import *

from zoef_msgs.srv import *

#import nest_asyncio
#nest_asyncio.apply()


loop = asyncio.get_event_loop()


# Map to convert from STM32 to pin numbers
# TODO: maybe convert this into JSON files and load it from there
# TODO: maybe just make this an list and get the index of the list
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
 "C13": 16,    # LED
 "C14": 17,
 "C15": 18,
 "A0" : 19,
 "A1" : 20,
 "A2" : 21,
 "A3" : 22,
 "A4" : 23,
 "A5" : 24,
 "A6" : 25,
 "A7" : 26,
 "B0" : 27,
 "B1" : 28,
 "B10": 29,
 "B11": 30
}
stm32_analog_offset = 14 #TODO: is this still needed in telemetrix?



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

def get_pin_numbers(component):
   devices = rospy.get_param('/zoef/device')
   device = devices[component["device"]]
   print(component)
   print(device)
   pins = {}
   if device["type"] == "zoef_pcb":
      mcu = "stm32"
      pins = zoef_pcb_map[component["connector"]]
   elif device["type"] == "breadboard":
      mcu = device["mcu"]
      pins = component["pins"]

   # convert pin naming to numbers
   # TODO: add support for nano
   print(pins)
   pin_numbers = {}
   for item in pins:
      if mcu == "stm32":
         pin_numbers[item] = stm32_map[pins[item]]
      if mcu == "nano":
         pin_numbers[item] = nano_map[pins[item]]
   print(pin_numbers)
   return pin_numbers

#TODO: move to main, and make sure board does not need to be global
rospy.init_node('zoef_telemetrix', anonymous=False)  # Only one node can connect with tty
device = 'zoef'
board = telemetrix_aio.TelemetrixAIO()

# Abstract Sensor class
class SensorMonitor:
    def __init__(self, board, pins, publisher, max_freq=100):
        self.board = board
        self.pins = pins
        self.publisher = publisher
        self.max_freq = max_freq
        self.loop = asyncio.get_event_loop()
        self.last_publish_time = -1

    def get_header(self):
       header = Header()
       header.stamp = rospy.Time.now()
       return header

    def publish(self, data):
       if self.max_freq == -1:
          self.publisher.publish(data)
       else:
          now_millis = int(round(time.time() * 1000))

          # always publish the first message (TODO: and mayeb messages that took too long 2x 1/freq?)
          if self.last_publish_time == -1:
             self.publisher.publish(data)
             self.last_publish_time = now_millis

          # from then on publish if needed based on max_freq
          if now_millis - self.last_publish_time >= 1000.0 / self.max_freq:
             self.publisher.publish(data)
             self.last_publish_time += (1000.0 / self.max_freq) # Note: this should not be set to now_millis. This is due to Nyquist.

class KeypadMonitor(SensorMonitor):
    def __init__(self, board, pins, pub, max_freq=100, differential=0):
        self.differential = differential
        super().__init__(board, pins, pub, max_freq=max_freq)
        self.last_debounce_time = 0
        self.last_key = ""
        self.last_debounced_key = ""
        self.pressed_publisher = rospy.Publisher('/zoef/keypad_pressed', Keypad, queue_size=1)

    async def start(self):
        await self.board.set_pin_mode_analog_input(self.pins - stm32_analog_offset, self.differential, self.publish_data)

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
       self.publisher.publish(keypad)

       # check if we need to send a pressed message
       if (self.last_debounced_key != "") and (self.last_debounced_key is not debounced_key):
          pressed = Keypad()
          pressed.header = self.get_header()
          pressed.key = self.last_debounced_key
          self.pressed_publisher.publish(pressed)

       self.last_key = key
       self.last_debounced_key = debounced_key

class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, pins, pub, max_freq=100):
        super().__init__(board, pins, pub, max_freq=max_freq)

    async def start(self):
        await self.board.set_pin_mode_sonar(self.pins["trigger"], self.pins["echo"], self.publish_data)

    async def publish_data(self, data):
       range = Range()
       range.header = self.get_header()
       range.radiation_type = range.ULTRASOUND
       range.field_of_view = math.pi * 5
       range.min_range = 0.02
       range.max_range = 1.5
       range.range = data[2]
       self.publisher.publish(range)

class IntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor, max_freq=100, differential=0):
        self.differential = differential
        pub = rospy.Publisher('/zoef/' + sensor, Intensity, queue_size=1)
        self.publisher_digital = rospy.Publisher('/zoef/' + sensor + "_digital", IntensityDigital, queue_size=1)
        intensity_sensors = rospy.get_param("/zoef/intensity")
        pins = get_pin_numbers(intensity_sensors[sensor])
        super().__init__(board, pins, pub, max_freq=max_freq)

    async def start(self):
       if self.pins["analog"]:
         await self.board.set_pin_mode_analog_input(self.pins["analog"] - stm32_analog_offset, differential=self.differential, callback=self.publish_analog_data)
       if self.pins["digital"]:
         await self.board.set_pin_mode_digital_input(self.pins["digital"], callback=self.publish_digital_data)

    async def publish_analog_data(self, data):
       intensity = Intensity()
       intensity.header = self.get_header()
       intensity.value = data[2]
       self.publish(intensity)

    async def publish_digital_data(self, data):
       intensity = IntensityDigital()
       intensity.header = self.get_header()
       intensity.value = bool(data[2])
       self.publisher_digital.publish(intensity)

class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, pins, pub, max_freq=100, differential=0, ticks_per_wheel=20):
        self.differential = differential
        self.ticks_per_wheel = ticks_per_wheel
        super().__init__(board, pins, pub, max_freq=max_freq)

    async def start(self):
        await self.board.set_pin_mode_encoder(self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data)

    async def publish_data(self, data):
       encoder = Encoder()
       encoder.header = self.get_header()
       encoder.value = data[2]
       self.publisher.publish(encoder)

#TODO: also supprt L298N
class MX1919Motor():
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_pwm = -1000 # why not 0?
        self.loop = asyncio.get_event_loop()
        for pin in self.pins:
            self.loop.run_until_complete(self.board.set_pin_mode_analog_output(pins[pin]))

    async def set_pwm(self, pwm):
        if (self.prev_motor_pwm != pwm):
          if (pwm >= 0):
            await self.board.analog_write(self.pins["1a"], 0)
            await self.board.analog_write(self.pins["1b"], min(abs(pwm) ,255))
          else:
            await self.board.analog_write(self.pins["1b"], 0)
            await self.board.analog_write(self.pins["1a"], min(abs(pwm) ,255))
          self.prev_motor_pwm = pwm

#class L298NMotor():
#    def __init__(self, board, pins):
#        self.board = board
#        self.pins = pins
#        self.prev_motor_pwm = -1000 # why not 0?
#        self.loop = asyncio.get_event_loop()
#        self.loop.run_until_complete(self.board.set_pin_mode_digital_output(pins[0]))
#        self.loop.run_until_complete(self.board.set_pin_mode_digital_output(pins[1]))
#        self.loop.run_until_complete(self.board.set_pin_mode_pwm_output(pins[2]))  # EN(able)
#
#    async def set_pwm(self, pwm):
#        if (self.prev_motor_pwm != pwm):
#          # First, set the pin to 0 (to make sure that not both pins are 1)
#          if (pwm >= 0):
#            await self.board.digital_write(self.pins[1], 0)
#            await self.board.digital_write(self.pins[0], 1)
#          else:
#            await self.board.digital_write(self.pins[0], 0)
#            await self.board.digital_write(self.pins[1], 1)
#          await self.board.pwm_write(self.pins[2], min(abs(pwm) ,255))
#          self.prev_motor_pwm = pwm


async def set_motor_pwm_service(req, motor):
    await motor.set_pwm(req.pwm)
    return SetMotorPWMResponse(True)

async def handle_set_led_value(req):
    led = rospy.get_param("/zoef/led")
    await board.analog_write(get_pin_numbers(led)["pin"], req.value)
    return SetLEDValueResponse(True)

async def handle_set_servo_angle(req):
    servo = rospy.get_param("/zoef/servo")
    await board.servo_write(get_pin_numbers(servo)["pin"], req.angle)
    return SetServoAngleResponse(True)

async def handle_get_pin_value(req):
  if req.type == "analog":
     data = await board.analog_read(req.pin)
     return GetPinValueResponse(data[0])
  if req.type == "digital":
     data = await board.digital_read(req.pin)
     return GetPinValueResponse(data[0])

async def handle_set_pin_value(req):
  if req.type == "analog":
     await board.set_pin_mode_analog_output(req.pin)
     return SetPinValueResponse(await board.analog_write(req.pin, req.value))
  if req.type == "digital":
     return SetPinValueResponse(await board.digital_write(req.pin, req.value))

async def shutdown(signal, loop, board):
    await board.shutdown()
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()
    exit(0)

#TODO: should these services become topics? and then be converted into services in teh other node?
def listener(loop, board):
    servers = []

    if rospy.has_param("/zoef/led"):
        led = rospy.get_param("/zoef/led")
        loop.run_until_complete(board.set_pin_mode_analog_output(get_pin_numbers(led)["pin"]))
        server = aiorospy.AsyncService('/zoef/set_led_value', SetLEDValue, handle_set_led_value)
        servers.append(loop.create_task(server.start()))

    if rospy.has_param("/zoef/servo"):
        servo = rospy.get_param("/zoef/servo")
        loop.run_until_complete(board.set_pin_mode_servo(get_pin_numbers(servo)["pin"]))
        server = aiorospy.AsyncService('/zoef/set_servo_angle', SetServoAngle, handle_set_servo_angle)
        servers.append(loop.create_task(server.start()))

    motors = {}
    if rospy.has_param("/zoef/motor"):
       motors = rospy.get_param("/zoef/motor")
       motors = {k: v for k, v in motors.items() if v['device'] == device}

    for motor in motors:
       # TODO: use variable in config file
#       if (len(motors[motor]['pin']) == 3):
#          motor_obj = L298NMotor(board, motors[motor]["pin"])
#       else:
       motor_obj = MX1919Motor(board, get_pin_numbers(motors[motor]))
       l = lambda req,m=motor_obj: set_motor_pwm_service(req, m)
       server = aiorospy.AsyncService("/zoef_pymata/set_" + motor + "_pwm", SetMotorPWM, l)
       servers.append(loop.create_task(server.start()))

    server = aiorospy.AsyncService('/zoef/set_pin_value', SetPinValue, handle_set_pin_value)
    server2 = aiorospy.AsyncService('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
    servers.append(loop.create_task(server.start()))
    servers.append(loop.create_task(server2.start()))

    for i in servers:
       loop.run_until_complete(i)

def publishers(loop):
   tasks = []
   max_freq = 250

   distance_sensors = {}
   if rospy.has_param("/zoef/distance"):
      distance_sensors = rospy.get_param("/zoef/distance")
      distance_sensors = {k: v for k, v in distance_sensors.items() if v['device'] == device}
      for sensor in distance_sensors:
         distance_publisher = rospy.Publisher('/zoef/' + sensor, Range, queue_size=1, latch=True)
         monitor = DistanceSensorMonitor(board, get_pin_numbers(distance_sensors[sensor]), distance_publisher, max_freq=100)
         tasks.append(loop.create_task(monitor.start()))

   intensity_sensors = {}
   if rospy.has_param("/zoef/intensity"):
      intensity_sensors = rospy.get_param("/zoef/intensity")
      intensity_sensors = {k: v for k, v in intensity_sensors.items() if v['device'] == device}
      for sensor in intensity_sensors:
         monitor = IntensitySensorMonitor(board, sensor, max_freq=100, differential=0)
         tasks.append(loop.create_task(monitor.start()))

   if rospy.has_param("/zoef/keypad"):
     keypad = rospy.get_param("/zoef/keypad")
     keypad_publisher = rospy.Publisher('/zoef/keypad', Keypad, queue_size=1)
     monitor = KeypadMonitor(board, get_pin_numbers(keypad)['pin'], keypad_publisher, max_freq=100, differential=0)
     tasks.append(loop.create_task(monitor.start()))

   encoder_sensors = {}
   if rospy.has_param("/zoef/encoder"):
      encoder_sensors = rospy.get_param("/zoef/encoder")
      encoder_sensors = {k: v for k, v in encoder_sensors.items() if v['device'] == device}
      for sensor in encoder_sensors:
         encoder_publisher = rospy.Publisher('/zoef/' + sensor, Encoder, queue_size=1, latch=True)
         monitor = EncoderSensorMonitor(board, get_pin_numbers(encoder_sensors[sensor]), encoder_publisher, max_freq=100, differential=0, ticks_per_wheel=encoder_sensors[sensor]["ticks_per_wheel"])
         tasks.append(loop.create_task(monitor.start()))

   for i in tasks:
      loop.run_until_complete(i)


   # For now, we need to set the analog scan interval to teh max_freq. When we set
   # this to 0, we do get the updates from telemetrix as fast as possible. In that
   # case the aiorospy creates a latency for the analog sensors (data will be
   # updated with a delay). This also happens when you try to implement this with
   # nest_asyncio icw rospy services.
   # Maybe there is a better solution for this, to make sure that we get the
   # data here asap.
   loop.run_until_complete(board.set_analog_scan_interval(int(1000.0/max_freq)))


if __name__ == '__main__':

   signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
   for s in signals:
      loop.add_signal_handler(s, lambda: asyncio.ensure_future(shutdown(s, loop, board)))

   publishers(loop)
   listener(loop, board)

   # Should not be: loop.run_forever() or rospy.spin()
   # Loop forever and give async some time to process
   # The sleep time should be lower than 1/max_freq
   # Since telemetrix updates at a max of 1ms, 0.00001
   # should be enough.
   while True:
      loop.run_until_complete(asyncio.sleep(0.00001))
