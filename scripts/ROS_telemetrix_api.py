#!/usr/bin/env python3.8
import asyncio
import os
import sys
import time
import math
import rospy
import signal
import aiorospy
from telemetrix_aio import telemetrix_aio

# Import ROS message types
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import *

# Import ROS services
from zoef_msgs.srv import *


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
stm32_max_pwm_value = 65535
stm32_analog_offset = 19

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
 "IR1"  : {"digital": "B10", "analog": "A0" },
 "IR2"  : {"digital": "B1" , "analog": "A1" },
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
if devices["zoef"]["type"] == "zoef_pcb" or devices["zoef"]["mcu"] == "stm32":
   analog_offset = stm32_analog_offset
   max_pwm_value = stm32_max_pwm_value
else:
   analog_offset = nano_analog_offset
   max_pwm_value = nano_max_pwm_value

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
      if mcu == "nano":
         pin_numbers[item] = nano_map[pins[item]]
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
        super().__init__(board, sensor, pub)
        self.ticks_per_wheel = sensor["ticks_per_wheel"]
        self.max_freq = -1
        self.last_publish_value = Encoder()

    def get_data(self, req):
        return GetEncoderResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_encoder(self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data)

    async def publish_data(self, data):
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

async def set_motor_speed_service(req, motor):
    await motor.set_speed(req.speed)
    return SetMotorSpeedResponse(True)

async def handle_set_led_value(req):
    led = rospy.get_param("/zoef/led")
    await board.analog_write(get_pin_numbers(led)["pin"], int(min(req.value, 100) / 100.0 * max_pwm_value))
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

    # TODO: support multiple leds
    if rospy.has_param("/zoef/led"):
       led = rospy.get_param("/zoef/led")
       loop.run_until_complete(board.set_pin_mode_analog_output(get_pin_numbers(led)["pin"]))
       server = aiorospy.AsyncService('/zoef/set_led_value', SetLEDValue, handle_set_led_value)
       servers.append(loop.create_task(server.start()))

    # TODO: support multiple servo's
    if rospy.has_param("/zoef/servo"):
        servo = rospy.get_param("/zoef/servo")
        loop.run_until_complete(board.set_pin_mode_servo(get_pin_numbers(servo)["pin"]))
        server = aiorospy.AsyncService('/zoef/set_servo_angle', SetServoAngle, handle_set_servo_angle)
        servers.append(loop.create_task(server.start()))

    if rospy.has_param("/zoef/motor"):
       motors = rospy.get_param("/zoef/motor")
       motors = {k: v for k, v in motors.items() if v['device'] == device}
       for motor in motors:
          motor_obj = PWMMotor(board, get_pin_numbers(motors[motor]))
          l = lambda req,m=motor_obj: set_motor_speed_service(req, m)
          server = aiorospy.AsyncService("/zoef/set_" + motor + "_speed", SetMotorSpeed, l)
          servers.append(loop.create_task(server.start()))

#    server = aiorospy.AsyncService('/zoef/set_pin_value', SetPinValue, handle_set_pin_value)
#    server2 = aiorospy.AsyncService('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
#    servers.append(loop.create_task(server.start()))
#    servers.append(loop.create_task(server2.start()))

    # Start all async tasks
    # TODO: explain why this will not work when they are started right after they
    # are created.
    for i in servers:
       loop.run_until_complete(i)

# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
   tasks = []
   max_freq = 0

   # initialze distance sensors
   if rospy.has_param("/zoef/distance"):
      distance_sensors = rospy.get_param("/zoef/distance")
      distance_sensors = {k: v for k, v in distance_sensors.items() if v['device'] == device}
      for sensor in distance_sensors:
         distance_publisher = rospy.Publisher('/zoef/' + sensor, Range, queue_size=1, latch=True)
         monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
         tasks.append(loop.create_task(monitor.start()))
         if "max_frequency" in distance_sensors[sensor] and distance_sensors[sensor]["max_frequency"] > max_freq:
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
         if "max_frequency" in intensity_sensors[sensor] and intensity_sensors[sensor]["max_frequency"] > max_freq:
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
         if "max_frequency" in keypad_sensors[sensor] and keypad_sensors[sensor]["max_frequency"] > max_freq:
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

   # Start all async tasks
   # TODO: explain why this will not work when they are started right after they
   # are created.
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
   board = telemetrix_aio.TelemetrixAIO()

   # Initialize the ROS node as anonymous since there
   # should only be one instnace running.
   rospy.init_node('zoef_telemetrix', anonymous=False, disable_signals=False)
   rospy.on_shutdown(send_sigint)

   device = 'zoef'
   sensors(loop, board, device)
   actuators(loop, board, device)

   # Should not be: loop.run_forever() or rospy.spin()
   # Loop forever and give async some time to process
   # The sleep time should be lower than 1/max_freq
   # Since telemetrix updates at a max of 1ms, 0.00001
   # should be enough.
   while True:
      loop.run_until_complete(asyncio.sleep(0.00001))
