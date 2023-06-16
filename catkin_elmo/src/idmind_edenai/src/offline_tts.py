#! /usr/bin/env python


import rospy
from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger
import requests
import os



"""

This module needs the installation of gTTS and mpg123 for speech generation and playback.

$ python3 -m pip install gTTS
$ sudo apt install mpg123

"""


class Node:

    def __init__(self):
        self.heartbeat_pub = rospy.Publisher(rospy.get_name() + "/heartbeat", Empty, queue_size=10)
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.language = rospy.get_param("~language")
        self.pending_input = None
        rospy.Subscriber(rospy.get_name() + "/input", String, self.on_input)
        self.speaking_pub = rospy.Publisher(rospy.get_name() + "/speaking", Bool, queue_size=1)
        rospy.Service(rospy.get_name() + "/enable", Trigger, self.on_enable)
        rospy.Service(rospy.get_name() + "/disable", Trigger, self.on_disable)

    def on_input(self, msg):
        data = msg.data
        forbidden = ["<", ">", "&", ";", "$", "|", "`", "(", ")", "{", "}", "[", "]", "\\", "!", "#", "%", "^", "*", "+", "=", "~", "?", ":", "'", "\"", ",", ".", "/", "@", "_"]
        for char in forbidden:
            data = data.replace(char, "")
        self.pending_input = data

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
            self.heartbeat_pub.publish()
            if not self.enabled:
                continue
            if not self.pending_input:
                continue
            self.speaking_pub.publish(True)
            command = "/usr/bin/rm -f /tmp/f.mp3 /tmp/f.wav && /home/idmind/.local/bin/gtts-cli -l %s \"%s\" --output /tmp/f.mp3 && /usr/bin/ffmpeg -i /tmp/f.mp3 /tmp/f.wav && aplay /tmp/f.wav && /usr/bin/rm -f /tmp/f.mp3 /tmp/f.wav" % (
                self.language,
                self.pending_input
            )
            print(command)
            os.system(command)
            self.speaking_pub.publish(False)
            self.pending_input = None


if __name__ == '__main__':
    rospy.init_node("text_to_speech")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
