#! /usr/bin/env python



import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import rosservice


STATE_LISTENING = 0
STATE_PROCESSING = 1
STATE_SPEAKING = 2


class Node:

    def __init__(self):
        self.state = STATE_LISTENING
        rospy.Subscriber("speech_to_text/output", String, self.on_speech_to_text)
        self.conversation_input_pub = rospy.Publisher("conversation/input", String, queue_size=10)
        rospy.Subscriber("conversation/output", String, self.on_conversation_output)
        self.text_to_speech_input_pub = rospy.Publisher(
            "text_to_speech/input",
            String,
            queue_size=1
        )
        rospy.Subscriber("text_to_speech/speaking", Bool, self.on_speaking)
        self.enable_speech_to_text_srv = None
    
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


    def on_speech_to_text(self, msg):
        print("got speech")
        if self.state == STATE_LISTENING:
            print("is listening")
            text = msg.data
            self.conversation_input_pub.publish(text)
            self.state = STATE_PROCESSING
            print("is now processing")

    def on_conversation_output(self, msg):
        print("got conversation")
        if self.state == STATE_PROCESSING or self.state == STATE_LISTENING:
            print("is processing")
            text = msg.data
            self.text_to_speech_input_pub.publish(text)
            self.state = STATE_SPEAKING
            print("is now speaking")

    def on_speaking(self, msg):
        speaking = msg.data
        if not speaking:
            print("no more speech")
            self.enable_speech_to_text()
            self.state = STATE_LISTENING
            print("is now listening")


if __name__ == '__main__':
    rospy.init_node("conversation_manager")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    rospy.spin()
