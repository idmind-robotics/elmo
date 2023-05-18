#! /usr/bin/env python


import rospy
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
import requests


class Node:

    def __init__(self):
        self.token = rospy.get_param("token")
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.pending_input = None
        rospy.Subscriber(rospy.get_name() + "/input", String, self.on_input)
        self.general_sentiment_pub = rospy.Publisher(rospy.get_name() + "/general_sentiment", String, queue_size=10)
        self.general_sentiment_rate_pub = rospy.Publisher(rospy.get_name() + "/general_sentiment_rate", Float64, queue_size=10)
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
            url = "https://api.edenai.run/v2/text/sentiment_analysis"
            payload = {
                "providers": "google",
                "language": "en",
                "text": self.pending_input
            }
            headers = {
                "Content-Type": "application/json",
                "Authorization": "Bearer %s" % self.token
            }
            response = requests.post(url, json=payload, headers=headers)
            data = response.json()
            general_sentiment = data["google"]["general_sentiment"]
            general_sentiment_rate = data["google"]["general_sentiment_rate"]
            self.general_sentiment_pub.publish(general_sentiment)
            self.general_sentiment_rate_pub.publish(general_sentiment_rate)
            self.pending_input = None


if __name__ == '__main__':
    rospy.init_node("sentiment_analysis")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
