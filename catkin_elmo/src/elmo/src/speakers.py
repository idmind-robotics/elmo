#! /usr/bin/env python3


import rospy
from std_msgs.msg import String, Int32
import os


RATE = 60


class Node:
    def __init__(self):
        rospy.Subscriber("speakers/url", String, self.on_url)
        rospy.Subscriber("speakers/volume", Int32, self.on_volume)

    def on_url(self, msg):
        url = msg.data
        os.system("curl %s | aplay" % url)

    def on_volume(self, msg):
        v = msg.data
        os.system("amixer sset 'Master' {}%".format(v))


if __name__ == '__main__':
    rospy.init_node("speakers")
    NODE = Node()
    rospy.loginfo("speakers: running")
    rospy.spin()
