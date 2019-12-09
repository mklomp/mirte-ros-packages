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
from zoef_msgs.msg import Encoder

from zoef_msgs.srv import *

board = PyMata("/dev/ttyUSB0", verbose=True)
rospy.init_node('listener', anonymous=True)

left_pub = rospy.Publisher('left_encoder', Encoder, queue_size=10)
right_pub = rospy.Publisher('right_encoder', Encoder, queue_size=10)


distance_sensors = rospy.get_param("/zoef/distance")
distance_publishers = {}
for sensor in distance_sensors:
   distance_publishers[sensor] = rospy.Publisher('/zoef/' + sensor, Range, queue_size=10)

prev_left_enc = False
prev_right_enc = False

def interrupt_callback2(data):
  global prev_left_enc
  curr_val = bool(data[2])
  if (prev_left_enc != curr_val):
    prev_left_enc = curr_val
    header = Header()
    header.stamp = rospy.Time.now()
    encoder = Encoder()
    encoder.header = header
    encoder.value = curr_val
    left_pub.publish(encoder)

def interrupt_callback3(data):
  global prev_right_enc
  curr_val = bool(data[2])
  if (prev_right_enc != curr_val):
    prev_left_enc = curr_val
    header = Header()
    header.stamp = rospy.Time.now()
    encoder = Encoder()
    encoder.header = header
    encoder.value = curr_val
    right_pub.publish(encoder)

prev_left = 0
prev_right = 0

# Create a PyMata instance
def init_pymata():

  def signal_handler(sig, frame):
    print('You pressed Ctrl+C')
    if board is not None:
        board.reset()
    sys.exit(0)

  for sensor in distance_sensors:
     board.sonar_config(distance_sensors[sensor]['pin'][0], distance_sensors[sensor]['pin'][1])

  signal.signal(signal.SIGINT, signal_handler)

  board.set_pin_mode(6, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(7, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(5, board.PWM, board.DIGITAL)

  board.set_pin_mode(8, board.OUTPUT, board.DIGITAL)
  #board.set_pin_mode(9, board.OUTPUT, board.DIGITAL)
  #board.set_pin_mode(10, board.PWM, board.DIGITAL)

  #TODO: make pymata.ino use in terrcupt on pin 2 and 3 and diable reporting on these pins
  # Or use encodrs like in pymata3
  board.set_pin_mode(2, board.INPUT, board.DIGITAL, interrupt_callback2)
  board.set_pin_mode(3, board.INPUT, board.DIGITAL, interrupt_callback3)

def left_callback(data):
    global prev_left
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (data.data != prev_left):
      if (data.data > 0):
	board.digital_write(7, 0)
        board.digital_write(6, 1)
      else: 
        board.digital_write(6, 0)
        board.digital_write(7, 1)
      board.analog_write(5, min(abs(data.data) * 3,255))
      prev_left = data.data

def right_callback(data):
    global prev_right
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (data.data != prev_right):
      if (data.data > 0):
        board.digital_write(9, 0)
        board.digital_write(8, 1)
      else: 
        board.digital_write(8, 0)
        board.digital_write(9, 1)
      board.analog_write(10, min(abs(data.data) * 3,255))
      prev_right = data.data

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


def listener():

    # Subscribers (actuators)
    rospy.Subscriber("left_pwm", Int32, left_callback, queue_size=1)
    rospy.Subscriber("right_pwm", Int32, right_callback, queue_size=1)

    # Services (raw arduino values)
    rospy.Service('get_pin_value', get_pin_value, handle_get_pin_value)

    # Timer publishers (sensors)
    for sensor in distance_sensors:
        l = lambda x,s=sensor: publish_distance(x, s)
        rospy.Timer(rospy.Duration(1.0/distance_sensors[sensor]['frequency']), l)

    rospy.spin()

if __name__ == '__main__':
    init_pymata()
    listener()
