#! /usr/bin/env python2.7

import time
import rospy
from std_msgs.msg import String, Int16

class Node(object):
    def __init__(self):
        self.eyes = rospy.Publisher("background", String, queue_size=10)
        rospy.Subscriber("blink", Int16, self.cb)

    def cb(self,data):
        self.n = 0
        while self.n < 7 :
            if data.data != 0:
                self.eyes.publish("normal")
                time.sleep(4.5)
                self.eyes.publish("sleep")
                time.sleep(0.2)
                self.eyes.publish("normal")
            else:
                self.n = 9
                break
        print("out")

if __name__ == '__main__':
    rospy.init_node("blink")
    NODE = Node()
    rospy.spin()