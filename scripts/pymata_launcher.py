#!/usr/bin/python

import rospy
import roslaunch

rospy.init_node('zoef_pymata_launcher', anonymous=False)
devices = rospy.get_param("/zoef/device")

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

for device in devices:
   package = 'zoef_ros_package'
   executable = 'pymata.py'
   node = roslaunch.core.Node(package, executable, "zoef_pymata_" + device, args="_device:=" + device, output="screen")
   process = launch.launch(node)

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.shutdown()
