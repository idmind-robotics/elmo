#! /usr/bin/env python3


import rospy
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
import requests
import json


import text2emotion as te


"""

This module depends on text2emotion which is fairly weird to install.

Install it using pip

$ python3 -m pip install text2emotion

Install a specific version of emoji

$ python3 -m pip install emoji==1.6.3

On some machines, aditional steps may be required.


"""


class Node:

    def __init__(self):
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.pending_input = None
        rospy.Subscriber(rospy.get_name() + "/input", String, self.on_input)
        self.happy_pub = rospy.Publisher(rospy.get_name() + "/happy", Float64, queue_size=10)
        self.angry_pub = rospy.Publisher(rospy.get_name() + "/angry", Float64, queue_size=10)
        self.surprise_pub = rospy.Publisher(rospy.get_name() + "/surprise", Float64, queue_size=10)
        self.sad_pub = rospy.Publisher(rospy.get_name() + "/sad", Float64, queue_size=10)
        self.fear_pub = rospy.Publisher(rospy.get_name() + "/fear", Float64, queue_size=10)
        self.json_pub = rospy.Publisher(rospy.get_name() + "/json", String, queue_size=10)
        rospy.Service(rospy.get_name() + "/enable", Trigger, self.on_enable)
        rospy.Service(rospy.get_name() + "/disable", Trigger, self.on_disable)

    def on_input(self, msg):
        self.pending_input = msg.data

    def on_enable(self, _):
        self.enabled = True
        self.pending_input = ""
        return True, "OK"

    def on_disable(self, _):
        self.enabled = False
        self.pending_input = ""
        return True, "OK"

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.enabled:
                continue
            if not self.pending_input:
                continue
            emotion = te.get_emotion(self.pending_input)
            self.json_pub.publish(json.dumps(emotion))
            happy = emotion['Happy']
            angry = emotion['Angry']
            surprise = emotion['Surprise']
            sad = emotion['Sad']
            fear = emotion['Fear']
            self.happy_pub.publish(happy)
            self.angry_pub.publish(angry)
            self.surprise_pub.publish(surprise)
            self.sad_pub.publish(sad)
            self.fear_pub.publish(fear)
            self.pending_input = None


if __name__ == '__main__':
    rospy.init_node("emotion_extraction")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
