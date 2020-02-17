#!/usr/bin/env python3.7
import asyncio
import sys
import time
import math
import rospy
import signal
import aiorospy
from pymata_express import pymata_express
from concurrent.futures import ProcessPoolExecutor

from std_msgs.msg import Int32, Empty, String, Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import Encoder, Intensity

from zoef_msgs.srv import *

#TODO: move to main, and make sure board does not need to be global
rospy.init_node('zoef_pymata', anonymous=True)
device = 'zoef'
#device = rospy.get_param('~device')
#devices = rospy.get_param("/zoef/device")
#dev = devices[device]['dev']
board = pymata_express.PymataExpress(baud_rate=1000000)

# Abstract Sensor class
class SensorMonitor:
    def __init__(self, board, pins, publisher, poll_freq=100):
        self.board = board
        self.pins = pins
        self.publisher = publisher
        self.poll_freq = poll_freq
        self.loop = asyncio.get_event_loop()

    def get_header(self):
       header = Header()
       header.stamp = rospy.Time.now()
       return header

class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, pins, pub, poll_freq=100):
        super().__init__(board, pins, pub, poll_freq=poll_freq)

    async def start(self):
        await self.board.set_pin_mode_sonar(self.pins[0], self.pins[1], self.publish_data)

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
    def __init__(self, board, pins, pub, poll_freq=100, differential=0):
        self.differential = differential
        super().__init__(board, pins, pub, poll_freq=poll_freq)

    async def start(self):
        await self.board.set_pin_mode_analog_input(self.pins[0], self.publish_data, differential=self.differential)

    async def publish_data(self, data):
       intensity = Intensity()
       intensity.header = self.get_header()
       intensity.value = data[2]
       self.publisher.publish(intensity)

class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, pins, pub, poll_freq=100, differential=0, ticks_per_wheel=20):
        self.differential = differential
        self.ticks_per_wheel = ticks_per_wheel
        super().__init__(board, pins, pub, poll_freq=poll_freq)

    async def start(self):
        await self.board.set_pin_mode_optenc(self.pins[0], self.ticks_per_wheel, 2, self.publish_data)

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
            self.loop.run_until_complete(self.board.set_pin_mode_pwm_output(pin))

    async def set_pwm(self, pwm):
        if (pwm >= 0):
          await self.board.pwm_write(self.pins[0], 0)
          await self.board.pwm_write(self.pins[1], min(abs(pwm) ,255))
        else:
          await self.board.pwm_write(self.pins[1], 0)
          await self.board.pwm_write(self.pins[0], min(abs(pwm) ,255))
        self.prev_motor_pwm = pwm

class L298NMotor():
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_pwm = -1000 # why not 0?
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.board.set_pin_mode_digital_output(pins[0]))
        self.loop.run_until_complete(self.board.set_pin_mode_digital_output(pins[1]))
        self.loop.run_until_complete(self.board.set_pin_mode_pwm_output(pins[2]))  # EN(able)

    async def set_pwm(self, pwm):
        # First, set the pin to 0 (to make sure that not both pins are 1)
        if (pwm >= 0):
          await self.board.digital_write(self.pins[1], 0)
          await self.board.digital_write(self.pins[0], 1)
        else:
          await self.board.digital_write(self.pins[0], 0)
          await self.board.digital_write(self.pins[1], 1)
        await self.board.pwm_write(self.pins[2], min(abs(pwm) ,255))
        self.prev_motor_pwm = pwm

async def set_motor_pwm_service(req, motor):
    await motor.set_pwm(req.pwm)
    return SetMotorPWMResponse(True)

async def handle_get_pin_value(req):
  if req.type == "analog":
     data = await board.analog_read(req.pin)
     return GetPinValueResponse(data[0])
  if req.type == "digital":
     data = await board.digital_read(req.pin)
     return GetPinValueResponse(data[0])

async def handle_set_pin_value(req):
  if req.type == "analog":
     await board.set_pin_mode_pwm_output(req.pin)
     return SetPinValueResponse(await board.pwm_write(req.pin, req.value))
  if req.type == "digital":
     return SetPinValueResponse(await board.digital_write(req.pin, req.value))

async def shutdown(signal, loop, board):
    await board.shutdown()
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()
    exit(0)

def listener():

    servers = []
    motors = {}
    if rospy.has_param("/zoef/motor"):
       motors = rospy.get_param("/zoef/motor")
       motors = {k: v for k, v in motors.items() if v['device'] == device}

    for motor in motors:
       # TODO: use variable in config file
       if (len(motors[motor]['pin']) == 3):
          motor_obj = L298NMotor(board, motors[motor]["pin"])
       else:
          motor_obj = MX1919Motor(board, motors[motor]["pin"])
       l = lambda req,m=motor_obj: set_motor_pwm_service(req, m)
       server = aiorospy.AsyncService("/zoef_pymata/set_" + motor + "_pwm", SetMotorPWM, l)
       servers.append(loop.create_task(server.start()))

    server = aiorospy.AsyncService('/zoef/set_pin_value', SetPinValue, handle_set_pin_value)
    server2 = aiorospy.AsyncService('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
    servers.append(loop.create_task(server.start()))
    servers.append(loop.create_task(server2.start()))

    for i in servers:
       loop.run_until_complete(i)

def publishers():
   loop = asyncio.get_event_loop()
   tasks = []

   distance_sensors = {}
   if rospy.has_param("/zoef/distance"):
      distance_sensors = rospy.get_param("/zoef/distance")
      distance_sensors = {k: v for k, v in distance_sensors.items() if v['device'] == device}
      for sensor in distance_sensors:
         distance_publisher = rospy.Publisher('/zoef/' + sensor, Range, queue_size=1)
         monitor = DistanceSensorMonitor(board, distance_sensors[sensor]['pin'], distance_publisher, poll_freq=-1)
         tasks.append(loop.create_task(monitor.start()))

   intensity_sensors = {}
   if rospy.has_param("/zoef/intensity"):
      intensity_sensors = rospy.get_param("/zoef/intensity")
      intensity_sensors = {k: v for k, v in intensity_sensors.items() if v['device'] == device}
      for sensor in intensity_sensors:
         intensity_publisher = rospy.Publisher('/zoef/' + sensor, Intensity, queue_size=1)
         monitor = IntensitySensorMonitor(board, [intensity_sensors[sensor]['pin']], intensity_publisher, poll_freq=-1, differential=0)
         tasks.append(loop.create_task(monitor.start()))

   encoder_sensors = {}
   if rospy.has_param("/zoef/encoder"):
      encoder_sensors = rospy.get_param("/zoef/encoder")
      encoder_sensors = {k: v for k, v in encoder_sensors.items() if v['device'] == device}
      for sensor in encoder_sensors:
         encoder_publisher = rospy.Publisher('/zoef/' + sensor, Encoder, queue_size=1)
         monitor = EncoderSensorMonitor(board, [encoder_sensors[sensor]['pin']], encoder_publisher, poll_freq=-1, differential=0, ticks_per_wheel=encoder_sensors[sensor]["ticks_per_wheel"])
         tasks.append(loop.create_task(monitor.start()))

   for i in tasks:
      loop.run_until_complete(i)



if __name__ == '__main__':
   loop = asyncio.get_event_loop()

   signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
   for s in signals:
      loop.add_signal_handler(s, lambda: asyncio.ensure_future(shutdown(s, loop, board)))

   loop.run_until_complete(board.set_sampling_interval(15)) #66Hz (pymata can go up to 1000Hz, but with ROS the CPU load becomes high and we get a lower max)
   publishers()
   listener()
   loop.run_forever() # same as rospy.spin()
