#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from zoef_msgs.srv import Move, MoveResponse, Turn, TurnResponse

velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

def handle_move(req):
    vel_msg = Twist()

    # Setting the current time for distance calculus
    vel_msg.linear.x = req.speed
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < req.distance):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = req.speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    return MoveResponse(True)

def handle_turn(req):
    vel_msg = Twist()

    # Setting the current time for distance calculus
    vel_msg.angular.z = req.speed
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < rq.angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    return TurnResponse(True)

def start_navigation_services():
    rospy.init_node('zoef_navigation', anonymous=False)
    move_service = rospy.Service('zoef_navigation/move', Move, handle_move)
    turn_service = rospy.Service('zoef_navigation/turn', Turn, handle_turn)
    rospy.spin()

if __name__ == "__main__":
    start_navigation_services()

