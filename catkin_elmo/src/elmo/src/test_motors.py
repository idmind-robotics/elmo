#! /usr/bin/env python3


import random
import rospy
import robot as r
from std_msgs.msg import String


MIN_SLEEP = 2.0
MAX_SLEEP = 4.0


class Node:

    def __init__(self):
        self.pan_tilt_api = r.PanTilt()
        self.enabled = False
        rospy.set_param("behaviour/test_motors/enabled", self.enabled)
        rospy.Subscriber("behaviour/enable", String, self.on_enable)
        rospy.Subscriber("behaviour/disable", String, self.on_disable)

    def on_enable(self, msg):
        if msg.data != "test_motors" or self.enabled:
            return
        print("enabling test motors")
        self.pan_tilt_api.enable(True, True)
        self.pan_tilt_api.reset_angles()
        self.enabled = True
        rospy.set_param("behaviour/test_motors/enabled", self.enabled)

    def on_disable(self, msg):
        if msg.data != "test_motors" or not self.enabled:
            return
        print("disabling test motors")
        self.enabled = False
        self.pan_tilt_api.reset_angles()
        rospy.sleep(2.0)
        self.pan_tilt_api.enable(False, False)
        rospy.set_param("behaviour/test_motors/enabled", self.enabled)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.enabled:
                limits = self.pan_tilt_api.get_limits()
                pan = random.uniform(limits["min_pan_angle"], limits["max_pan_angle"])
                tilt = random.uniform(limits["min_tilt_angle"], limits["max_tilt_angle"])
                self.pan_tilt_api.set_angles(
                    pan=pan,
                    tilt=tilt
                )
                rospy.loginfo("pan: %.2f | tilt: %.2f" % (pan, tilt) )
                rospy.sleep(random.uniform(MIN_SLEEP, MAX_SLEEP))


if __name__ == '__main__':
    rospy.init_node("test_motors")
    NODE = Node()
    rospy.loginfo("test_motors: running")
    NODE.run()
