#! /usr/bin/env python


import rospy
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
import requests


INITIAL_CONTEXT_EN = """

Human: Pretend that you are a robot named Elmo. Include in your replies only text.

Robot: OK"""


INITIAL_CONTEXT_PT = """

Human: Pretend that you are a robot named Elmo. Include in your replies only text. Reply in portuguese from Portugal.

Robot: OK"""


CONTEXT_RESET_TIMEOUT = 60 * 5


class Node:

    def __init__(self):
        self.heartbeat_pub = rospy.Publisher(rospy.get_name() + "/heartbeat", Empty, queue_size=10)
        self.token = rospy.get_param("token")
        self.language = rospy.get_param("language", "en")
        self.initial_context = INITIAL_CONTEXT_EN if self.language == "en" else INITIAL_CONTEXT_PT
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.max_tokens = rospy.get_param(rospy.get_name() + "/max_tokens", 100)
        self.temperature = rospy.get_param(rospy.get_name() + "/temperature", 0.1)
        self.context = self.initial_context
        self.pending_input = None
        self.last_input_time = rospy.Time.now()
        rospy.Subscriber(rospy.get_name() + "/input", String, self.on_input)
        self.pub = rospy.Publisher(rospy.get_name() + "/output", String, queue_size=10)
        rospy.Service(rospy.get_name() + "/reset_context", Trigger, self.on_reset_context)
        rospy.Service(rospy.get_name() + "/enable", Trigger, self.on_enable)
        rospy.Service(rospy.get_name() + "disable", Trigger, self.on_disable)

    def on_input(self, msg):
        self.last_input_time = rospy.Time.now()
        self.pending_input = msg.data

    def on_reset_context(self, _):
        self.reset_context()
        return True, "OK"

    def reset_context(self):
        if self.context == self.initial_context:
            return
        rospy.loginfo("resetting context")
        self.context = self.initial_context

    def on_enable(self, _):
        self.enabled = True
        self.context = self.initial_context
        self.pending_input = ""
        return True, "OK"

    def on_disable(self, _):
        self.enabled = False
        self.context = self.initial_context
        self.pending_input = ""
        return True, "OK"

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            self.heartbeat_pub.publish()
            if not self.enabled:
                continue
            if (rospy.Time.now() - self.last_input_time).to_sec() > CONTEXT_RESET_TIMEOUT:
                self.reset_context()
            if not self.pending_input:
                continue
            data = self.context + "\n\nHuman: " + self.pending_input
            self.context += "\n\nHuman: " + self.pending_input
            url = "https://api.edenai.run/v2/text/generation"
            payload = {
                "providers": "openai",
                "text": data,
                "temperature": 0.1,
                "max_tokens": self.max_tokens,
                "model": None,
                "show_original_response": True
            }
            headers = {
                "Content-Type": "application/json",
                "Authorization": "Bearer %s" % self.token
            }
            print("---")
            print(self.context)
            response = requests.post(url, json=payload, headers=headers)
            data = response.json()
            print(data)
            generated_text = data["openai"]["generated_text"].encode("utf-8")
            reply = "".join(generated_text.split(":")[1:])
            print("generated text: " + generated_text)
            print(reply)
            self.context += generated_text
            self.pub.publish(reply)
            self.pending_input = None
            print(self.context)


if __name__ == '__main__':
    rospy.init_node("conversation")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
