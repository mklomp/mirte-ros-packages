#!/usr/bin/python

# This node listens in on teh sensor publishers, and stores the last value to be used
# in the service call.
# This is done in order to use topics on the ROS side, but only make (blocking) calls
# from the python api (robot.py)

import rospy
import message_filters
from sensor_msgs.msg import Range
from zoef_msgs.msg import Intensity
from zoef_msgs.srv import GetDistance, GetDistanceResponse, GetIntensity, GetIntensityResponse

# Message filters
distance_sensors = rospy.get_param("/zoef/distance")
distance_caches = {}
for sensor in distance_sensors:
   distance_filter = message_filters.Subscriber('/zoef/' + sensor, Range)
   distance_caches[sensor] = message_filters.Cache(distance_filter, 1)

intensity_sensors = rospy.get_param("/zoef/intensity")
intensity_caches = {}
for sensor in intensity_sensors:
   intensity_filter = message_filters.Subscriber('/zoef/' + sensor, Intensity)
   intensity_caches[sensor] = message_filters.Cache(intensity_filter, 1)


def handle_distance(req, sensor):
    now = rospy.get_rostime()
    last_value = distance_caches[sensor].getElemBeforeTime(now)
    return GetDistanceResponse(last_value.range)

def handle_intensity(req, sensor):
    now = rospy.get_rostime()
    last_value = intensity_caches[sensor].getElemBeforeTime(now)
    return GetIntensityResponse(last_value.value)

def start_service_api():
    rospy.init_node('zoef_service_api', anonymous=False)

    distance_sensors = rospy.get_param("/zoef/distance")
    for sensor in distance_sensors:
       l = lambda msg, s=sensor: handle_distance(msg, s)
       rospy.Service('/zoef_service_api/get_' + sensor, GetDistance, l)

    intensity_sensors = rospy.get_param("/zoef/intensity")
    for sensor in intensity_sensors:
       l = lambda msg, s=sensor: handle_intensity(msg, s)
       rospy.Service('/zoef_service_api/get_' + sensor, GetIntensity, l)

    rospy.spin()

if __name__ == "__main__":
    start_service_api()

