#!/usr/bin/python

# This node listens in on teh sensor publishers, and stores the last value to be used
# in the service call.
# This is done in order to use topics on the ROS side, but only make (blocking) calls
# from the python api (robot.py)

import rospy
import message_filters
from sensor_msgs.msg import Range
from zoef_msgs.srv import GetDistance, GetDistanceResponse

# Message filters
distance_filter = message_filters.Subscriber('distance', Range)
distance_cache = message_filters.Cache(distance_filter, 1, allow_headerless=True)

def handle_distance(req):
    now = rospy.get_rostime()
    last_value = distance_cache.getElemBeforeTime(now)
    return GetDistanceResponse(last_value.range)

def start_service_api():
    rospy.init_node('zoef_service_api', anonymous=False)
    move_service = rospy.Service('zoef_service_api/get_distance', GetDistance, handle_distance)
    rospy.spin()

if __name__ == "__main__":
    start_service_api()

