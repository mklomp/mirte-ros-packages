#!/usr/bin/python

import rospy
import time
import sys
import signal

from PyMata.pymata import PyMata
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String

board = PyMata("/dev/ttyUSB0", verbose=True)
rospy.init_node('listener', anonymous=True)

left_pub = rospy.Publisher('left_encoder', String, queue_size=10)
right_pub = rospy.Publisher('right_encoder', String, queue_size=10)
distance_pub = rospy.Publisher('distance', Int32, queue_size=10)

trigger_pin = 12
echo_pin = 11


def interrupt_callback2(data):
	left_pub.publish(str(data[2]))

def interrupt_callback3(data):
        right_pub.publish(str(data[2]))

prev_left = 0
prev_right = 0


# Create a PyMata instance
def init_pymata():

  def signal_handler(sig, frame):
    print('You pressed Ctrl+C')
    if board is not None:
        board.reset()
    sys.exit(0)

#  board.set_pin_mode(3, board.OUTPUT, board.DIGITAL)
#  board.set_pin_mode(4, board.OUTPUT, board.DIGITAL)
  board.sonar_config(trigger_pin, echo_pin)

  signal.signal(signal.SIGINT, signal_handler)

  board.set_pin_mode(6, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(7, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(5, board.PWM, board.DIGITAL)

  board.set_pin_mode(8, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(9, board.OUTPUT, board.DIGITAL)
  board.set_pin_mode(10, board.PWM, board.DIGITAL)

  #TODO: make pymata.ino use in terrcupt on pin 2 and 3 and diable reporting on these pins
  board.set_pin_mode(2, board.INPUT, board.DIGITAL, interrupt_callback2)
  board.set_pin_mode(3, board.INPUT, board.DIGITAL, interrupt_callback3)



def left_callback(data):
#    global board
    global prev_left
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (data.data != prev_left):
      if (data.data > 0):
	board.digital_write(7, 0)
        board.digital_write(6, 1)
	print "left forward"
      else: 
        board.digital_write(6, 0)
        board.digital_write(7, 1)
	print "left backward"
      board.analog_write(5, min(abs(data.data) * 3,255))
      prev_left = data.data

def right_callback(data):
#    global board
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


def distance_callback(data):
	sonar = board.get_sonar_data()
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(sonar[trigger_pin]))
	dist = Int32()
	dist.data = sonar[trigger_pin][1]
	distance_pub.publish(dist)


def listener():
    rospy.Subscriber("left_pwm", Int32, left_callback, queue_size=1)
    rospy.Subscriber("right_pwm", Int32, right_callback, queue_size=1)

    rospy.Subscriber("distance_request", Empty, distance_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    init_pymata()
    listener()



