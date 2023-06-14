#! /usr/bin/env python


import rospy
from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger
import requests
from playsound import playsound
import os


class Node:

    def __init__(self):
        self.heartbeat_pub = rospy.Publisher(rospy.get_name() + "/heartbeat", Empty, queue_size=10)
        self.token = rospy.get_param("token")
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.pending_input = None
        rospy.Subscriber(rospy.get_name() + "/input", String, self.on_input)
        self.speaking_pub = rospy.Publisher(rospy.get_name() + "/speaking", Bool, queue_size=1)
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
            self.heartbeat_pub.publish()
            if not self.enabled:
                continue
            if not self.pending_input:
                continue
            url = "https://api.edenai.run/v2/audio/text_to_speech"
            payload = {
                "providers": "lovoai",
                "language": "en",
                "text": self.pending_input,
                "option": "MALE",
                "settings": {
                    'lovoai': 'en-US_Austin Hopkins'
                }
            }
            headers = {
                "Content-Type": "application/json",
                "Authorization": "Bearer %s" % self.token
            }
            response = requests.post(url, json=payload, headers=headers)
            data = response.json()
            print(data)
            output_url = data["lovoai"]["audio_resource_url"]
            print(output_url)
            # playsound(output_url)
            self.speaking_pub.publish(True)
            os.system("curl \"%s\" | aplay" % output_url)
            self.speaking_pub.publish(False)
            self.pending_input = None


if __name__ == '__main__':
    rospy.init_node("text_to_speech")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
