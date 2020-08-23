#!/usr/bin/env python

import rospy
# from std_msgs.msg import UInt16
from raspimouse_ros_2.msg import *

class pimouse_sim_run():
    def __init__(self):
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        self.data = LightSensorValues()

    def sensor_callback(self, msg):
        self.data = msg

# -----

    def motion(self):
        print(self.data.right_forward)
        print(self.data.right_side)
        print(self.data.left_side)
        print(self.data.left_forward)

# -----

    def run(self):
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pimouse_sim_run')
    pimouse_sim_run().run()
