#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from mirte_msgs.srv import Move, MoveResponse, Turn, TurnResponse

velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

def handle_move(req):
    # req.distance = distance (directional) in m
    # req.speed = speed (always positive) in m/s
    if (req.speed < 0):
       return MoveResponse(False)

    vel_msg = Twist()

    # Setting the current time for distance calculus
    directional_speed = req.speed
    if req.distance < 0:
       directional_speed = -req.speed
    vel_msg.linear.x = directional_speed
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # TODO: not calculating, but using odom
    while(current_distance < abs(req.distance)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = req.speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    return MoveResponse(True)

def handle_turn(req):
    # req.angle = angle (directional, positive is clockwise) in rad
    # req.speed = angular speed (always positive) in rad/s
    if (req.speed < 0):
       return TurnResponse(False)

    vel_msg = Twist()

    # Setting the current time for distance calculus
    directional_speed = req.speed
    if req.angle < 0:
       directional_speed = -req.speed
    vel_msg.angular.z = directional_speed
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    # TODO: not calculating, but from odom
    while(current_angle < abs(req.angle)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = req.speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    return TurnResponse(True)

def start_navigation_services():
    rospy.init_node('mirte_navigation', anonymous=False)
    move_service = rospy.Service('mirte_navigation/move', Move, handle_move)
    turn_service = rospy.Service('mirte_navigation/turn', Turn, handle_turn)
    rospy.spin()

if __name__ == "__main__":
    start_navigation_services()

