#!/usr/bin/python

import rospy
import time
import sys
import signal

from PyMata.pymata import PyMata
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Header
from zoef_msgs.msg import Encoder

from zoef_msgs.srv import *

board = PyMata("/dev/ttyUSB0", verbose=True)
rospy.init_node('listener', anonymous=True)

left_pub = rospy.Publisher('left_encoder', Encoder, queue_size=10)
right_pub = rospy.Publisher('right_encoder', Encoder, queue_size=10)

distance_publisher = rospy.Publisher('distance', Int32, queue_size=10)


trigger_pin = 12
echo_pin = 11

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

  board.sonar_config(trigger_pin, echo_pin)

  signal.signal(signal.SIGINT, signal_handler)

  board.set_pin_mode(6, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(7, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(5, board.PWM, board.DIGITAL)

  board.set_pin_mode(8, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(9, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(10, board.PWM, board.DIGITAL)

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
def publish_distance(timer):
    sonar = board.get_sonar_data()
    distance_publisher.publish(sonar[trigger_pin][1])


def listener():
    global ticks
    # Subscribers (actuators)
    rospy.Subscriber("left_pwm", Int32, left_callback, queue_size=1)
    rospy.Subscriber("right_pwm", Int32, right_callback, queue_size=1)

    # Services (raw arduino values)
    rospy.Service('get_pin_value', get_pin_value, handle_get_pin_value)

    # Publishers (sensors) using timers
    distance_rate = 10 # TODO: set via dynamic reconfigure)
    rospy.Timer(rospy.Duration(1.0 / distance_rate), publish_distance)

    rospy.spin()

if __name__ == '__main__':
    init_pymata()
    listener()
