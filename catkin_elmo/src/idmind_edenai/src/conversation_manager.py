#! /usr/bin/env python



import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import rosservice

import logger


STATE_LISTENING = 0
STATE_PROCESSING = 1
STATE_SPEAKING = 2


class Node:

    def __init__(self):
        self.logger = logger.Logger()
        self.state = STATE_LISTENING
        rospy.Subscriber("speech_to_text/output/en", String, self.on_speech_to_text)
        self.conversation_input_pub = rospy.Publisher("conversation/input", String, queue_size=10)
        rospy.Subscriber("conversation/output/pt", String, self.on_conversation_output)
        self.text_to_speech_input_pub = rospy.Publisher(
            "text_to_speech/input",
            String,
            queue_size=1
        )
        rospy.Subscriber("text_to_speech/speaking", Bool, self.on_speaking)
        self.enable_speech_to_text_srv = None
        self.disable_speech_to_text_srv = None
    
    def enable_speech_to_text(self):
        if self.enable_speech_to_text_srv is not None:
            print("enabling speech to text")
            self.enable_speech_to_text_srv()
        else:
            if "speech_to_text/enable" in rosservice.get_service_list():
                self.enable_speech_to_text_srv = rospy.ServiceProxy("speech_to_text/enable", Trigger)
                print("enabling speech to text")
                self.enable_speech_to_text_srv()
            else:
                print("service speech_to_text/enable is unavailable, ignoring.")

    def disable_speech_to_text(self):
        if self.disable_speech_to_text_srv is not None:
            print("disabling speech to text")
            self.disable_speech_to_text_srv()
        else:
            if "speech_to_text/disable" in rosservice.get_service_list():
                self.disable_speech_to_text_srv = rospy.ServiceProxy("speech_to_text/disable", Trigger)
                print("disabling speech to text")
                self.disable_speech_to_text_srv()
            else:
                print("service speech_to_text/disable is unavailable, ignoring.")


    def on_speech_to_text(self, msg):
        print("got speech")
        if self.state == STATE_LISTENING:
            self.logger.info("is listening")
            self.disable_speech_to_text()
            text = msg.data
            self.conversation_input_pub.publish(text)
            self.state = STATE_PROCESSING
            self.logger.info("is now processing")

    def on_conversation_output(self, msg):
        print("got conversation")
        if self.state == STATE_PROCESSING or self.state == STATE_LISTENING:
            self.logger.info("is processing")
            text = msg.data
            self.text_to_speech_input_pub.publish(text)
            self.state = STATE_SPEAKING
            self.logger.info("is now speaking")

    def on_speaking(self, msg):
        speaking = msg.data
        if not speaking:
            self.logger.info("no more speech")
            rospy.sleep(2.0)
            self.enable_speech_to_text()
            self.state = STATE_LISTENING
            self.logger.info("is now listening")


if __name__ == '__main__':
    rospy.init_node("conversation_manager")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    rospy.spin()
