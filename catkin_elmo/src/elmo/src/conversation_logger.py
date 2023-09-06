#! /usr/bin/env python3



"""

This node logs conversations to a file.

Conversations are stored in /home/idmind/conversations/ and are named after the current date.

Subscribes to what people say to the robot via the topic /conversation/input.

Subscribes to what the robot says via the topic /conversation/output.

Conversations are stored in the following format:

    [<current time>] Human: <what the human said>
    [<current time>] Robot: <what the robot said>
    ...

Listens to /conversation/reset to close the current conversation.

When a conversation is closed, appends "---" to the file.

"""

import rospy
from std_msgs.msg import String, Empty
import datetime



class Node:

    def __init__(self):
        self.writing = False
        self.outdir = "/home/idmind/conversations/"
        rospy.Subscriber("/conversation/input", String, self.on_input)
        rospy.Subscriber("/conversation/output", String, self.on_output)
        rospy.Subscriber("/conversation/reset", Empty, self.on_reset)
        filename = self.outdir + datetime.datetime.now().strftime("%Y-%m-%d") + ".txt"
        with open(filename, "a") as f:
            f.write("---\n")
    
    def on_input(self, msg):
        filename = self.outdir + datetime.datetime.now().strftime("%Y-%m-%d") + ".txt"
        current_time = datetime.datetime.now().strftime("[%H:%M:%S]")
        while self.writing:
            pass
        self.writing = True
        with open(filename, "a") as f:
            f.write(current_time + " Human: " + msg.data + "\n")
        self.writing = False

    def on_output(self, msg):
        filename = self.outdir + datetime.datetime.now().strftime("%Y-%m-%d") + ".txt"
        current_time = datetime.datetime.now().strftime("[%H:%M:%S]")
        while self.writing:
            pass
        self.writing = True
        with open(filename, "a") as f:
            f.write(current_time + " Robot: " + msg.data + "\n")
        self.writing = False

    def on_reset(self, _):
        filename = self.outdir + datetime.datetime.now().strftime("%Y-%m-%d") + ".txt"
        while self.writing:
            pass
        self.writing = True
        with open(filename, "a") as f:
            f.write("---\n")
        self.writing = False


if __name__ == "__main__":
    rospy.init_node("conversation_logger")
    node = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    rospy.spin()
