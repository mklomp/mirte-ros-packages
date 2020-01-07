#!/usr/bin/python

import rospy
import time
import sys
import signal
import math

from PyMata.pymata import PyMata
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import Encoder, Intensity

from zoef_msgs.srv import *

rospy.init_node('zoef_pymata', anonymous=True)
device = rospy.get_param('~device')
devices = rospy.get_param("/zoef/device")
dev = devices[device]['dev']
board = PyMata(dev, verbose=True, baud_rate=1000000)
board.set_sampling_interval(100)

distance_sensors = rospy.get_param("/zoef/distance")
distance_sensors = {k: v for k, v in distance_sensors.iteritems() if v['device'] == device}
distance_publishers = {}
for sensor in distance_sensors:
   distance_publishers[sensor] = rospy.Publisher('/zoef/' + sensor, Range, queue_size=10)

intensity_sensors = rospy.get_param("/zoef/intensity")
intensity_sensors = {k: v for k, v in intensity_sensors.iteritems() if v['device'] == device}
intensity_publishers = {}
for sensor in intensity_sensors:
   intensity_publishers[sensor] = rospy.Publisher('/zoef/' + sensor, Intensity, queue_size=10)

encoder_sensors = rospy.get_param("/zoef/encoder")
encoder_sensors = {k: v for k, v in encoder_sensors.iteritems() if v['device'] == device}
encoder_publishers = {}
for sensor in encoder_sensors:
   encoder_publishers[sensor] = rospy.Publisher('/zoef/' + sensor, Encoder, queue_size=10)

motors = rospy.get_param("/zoef/motor")
motors = {k: v for k, v in motors.iteritems() if v['device'] == device}
prev_motor_pwm = {}
for motor in motors:
   prev_motor_pwm[motor] = 0


def publish_encoder(data, sensor):
    header = Header()
    header.stamp = rospy.Time.now()
    encoder = Encoder()
    encoder.header = header
    encoder.value = data[2][1]
    encoder_publishers[sensor].publish(encoder)

# Create a PyMata instance
def init_pymata():

  def signal_handler(sig, frame):
    print('You pressed Ctrl+C')
    if board is not None:
        board.reset()
    sys.exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  # TODO: store a list of set values, so they can not be changed
  for sensor in distance_sensors:
     board.sonar_config(distance_sensors[sensor]['pin'][0], distance_sensors[sensor]['pin'][1])

  for sensor in intensity_sensors:
     board.set_pin_mode(intensity_sensors[sensor]['pin'], board.INPUT, board.ANALOG)

  for motor in motors:
     board.set_pin_mode(motors[motor]['pin'][0], board.OUTPUT, board.DIGITAL)   #6, 8
     board.set_pin_mode(motors[motor]['pin'][1], board.OUTPUT, board.DIGITAL)   #7, 9
     board.set_pin_mode(motors[motor]['pin'][2], board.PWM, board.DIGITAL)      #5,10

  for sensor in encoder_sensors:
     sensor_args = encoder_sensors[sensor]
     l = lambda x,s=sensor: publish_encoder(x, s)
     board.optical_encoder_config(sensor_args['pin'], sensor_args['ticks_per_wheel'], cb=l)

# Set PWM values
def set_motor_pwm(req, motor):
    global prev_motor_pwm
    if (req.pwm != prev_motor_pwm[motor]):
      if (req.pwm > 0):
        board.digital_write(motors[motor]['pin'][1], 0)
        board.digital_write(motors[motor]['pin'][0], 1)
      else:
        board.digital_write(motors[motor]['pin'][0], 0)
        board.digital_write(motors[motor]['pin'][1], 1)
      board.analog_write(motors[motor]['pin'][2], min(abs(req.pwm) ,255))
      prev_motor_pwm[motor] = req.pwm
    return SetMotorPWMResponse(True)

def handle_get_pin_value(req):
  board.set_pin_mode(req.pin, board.INPUT, board.ANALOG)
  return get_pin_valueResponse(board.analog_read(req.pin))

# Publish distance sensor
def publish_distance(timer, sensor):
    sonar = board.get_sonar_data()
    trigger_pin = distance_sensors[sensor]['pin'][0]
    dist_value = sonar[trigger_pin][1]
    if isinstance(dist_value, list): # sometimes this returns as a array (TODO: find out where that comes from)
       dist_value = dist_value[0]
    header = Header()
    header.stamp = rospy.Time.now()
    range = Range()
    range.header = header
    range.radiation_type = range.ULTRASOUND
    range.field_of_view = math.pi * 5
    range.min_range = 0.02
    range.max_range = 1.5
    range.range = dist_value
    distance_publishers[sensor].publish(range)

# Publish (IR) light intensity sensor
def publish_intensity(timer, sensor):
    pin = intensity_sensors[sensor]['pin']
    intensity_value = board.analog_read(pin)
    header = Header()
    header.stamp = rospy.Time.now()
    intensity = Intensity()
    intensity.header = header
    intensity.value = intensity_value
    intensity_publishers[sensor].publish(intensity)


def listener():

    # Services (actuators)
    for motor in motors:
       l = lambda req,m=motor: set_motor_pwm(req, m)
       rospy.Service("/zoef_pymata/set_" + motor + "_pwm", SetMotorPWM, l)

    # Services (raw arduino values)
    rospy.Service('get_pin_value', get_pin_value, handle_get_pin_value)

    # Timer publishers (sensors)
    for sensor in distance_sensors:
        l = lambda x,s=sensor: publish_distance(x, s)
        rospy.Timer(rospy.Duration(1.0/distance_sensors[sensor]['frequency']), l)

    for sensor in intensity_sensors:
        l = lambda x,s=sensor: publish_intensity(x, s)
        rospy.Timer(rospy.Duration(1.0/intensity_sensors[sensor]['frequency']), l)

    rospy.spin()

if __name__ == '__main__':
    init_pymata()
    listener()
