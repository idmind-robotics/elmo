#! /usr/bin/env python



import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import rosservice

import logger
import robot


LOOP_RATE = 10

STATE_LISTENING = 0
STATE_PROCESSING = 1
STATE_SPEAKING = 2

START_SPEAKING_TIMEOUT = 5.0


class Node:

    def __init__(self):
        self.logger = logger.Logger()
        self.onboard = robot.Onboard()
        server = robot.Server()
        self.normal_url = server.url_for_image("normal.png")
        self.thinking_url = server.url_for_image("thinking2.png")
        self.state = STATE_LISTENING
        # rospy.Subscriber("speech_to_text/output/en", String, self.on_speech_to_text)
        rospy.Subscriber("speech_to_text/output", String, self.on_speech_to_text)
        self.conversation_input_pub = rospy.Publisher("conversation/input", String, queue_size=10)
        # rospy.Subscriber("conversation/output/pt", String, self.on_conversation_output)
        rospy.Subscriber("conversation/output", String, self.on_conversation_output)
        self.text_to_speech_input_pub = rospy.Publisher(
            "text_to_speech/input",
            String,
            queue_size=1
        )
        rospy.Subscriber("text_to_speech/speaking", Bool, self.on_speaking)
        self.sent_speech_at = rospy.Time.now()
        self.speaking = False

    def on_speech_to_text(self, msg):
        print("got speech")
        if self.state == STATE_LISTENING:
            self.logger.info("is listening")
            text = msg.data
            self.conversation_input_pub.publish(text)
            self.state = STATE_PROCESSING
            self.logger.info("is now processing")
            self.onboard.set_image(self.thinking_url)

    def on_conversation_output(self, msg):
        print("got conversation")
        if self.state == STATE_PROCESSING or self.state == STATE_LISTENING:
            self.logger.info("is processing")
            text = msg.data
            self.text_to_speech_input_pub.publish(text)
            self.state = STATE_SPEAKING
            self.logger.info("is now speaking")
            self.onboard.set_image(self.normal_url)
            self.sent_speech_at = rospy.Time.now()

    def on_speaking(self, msg):
        speaking = msg.data
        if not speaking:
            self.logger.info("no more speech")
            rospy.sleep(2.0)
            self.state = STATE_LISTENING
            self.logger.info("is now listening")
        self.speaking = speaking

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            # check sent speech timeout
            if self.state == STATE_SPEAKING:
                if not self.speaking:
                    self.logger.info("waiting for speech")
                    if (rospy.Time.now() - self.sent_speech_at).to_sec() > START_SPEAKING_TIMEOUT:
                        self.logger.info("speech timeout")
                        self.state = STATE_LISTENING
                        self.logger.info("is now listening")


if __name__ == '__main__':
    rospy.init_node("conversation_manager")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
