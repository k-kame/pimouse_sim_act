#!/usr/bin/env python

import rospy
# from std_msgs.msg import UInt16
from raspimouse_ros_2.msg import *

class pimouse_sim_run():
    def __init__(self):
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        self.data = LightSensorValues()

    def sensor_callback(self, meg):
        rospy.loginfo(type(self.data))

# -----

    def motion(self):
        # self.rate.sleep()
        rospy.loginfo(type(self.data))
        rospy.loginfo(self.data)

# -----

    def run(self):
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pimouse_sim_run')
    pimouse_sim_run().run()
