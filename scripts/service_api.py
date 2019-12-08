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
distance_sensors = rospy.get_param("/zoef/distance")
distance_caches = {}
for sensor in distance_sensors:
   distance_filter = message_filters.Subscriber('/zoef/' + sensor, Range)
   distance_caches[sensor] = message_filters.Cache(distance_filter, 1)

def handle_distance(req, sensor):
    now = rospy.get_rostime()
    print sensor
    last_value = distance_caches[sensor].getElemBeforeTime(now)
    return GetDistanceResponse(last_value.range)

def start_service_api():
    rospy.init_node('zoef_service_api', anonymous=False)
    distance_sensors = rospy.get_param("/zoef/distance")
    move_services = {}
    for sensor in distance_sensors:
       l = lambda msg, s=sensor: handle_distance(msg, s)
       move_services[sensor] = rospy.Service('zoef_service_api/get_' + sensor, GetDistance, l)

    rospy.spin()

if __name__ == "__main__":
    start_service_api()

