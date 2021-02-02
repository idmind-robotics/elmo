#! /usr/bin/env python2.7

import rospy
from std_msgs.msg import String, Int16
import time

l=[]

class Node(object):
    
    def __init__(self):
        rospy.Subscriber("presence", Int16, self.sleep)
        self.blink = rospy.Publisher("blink", Int16)
        self.eyes = rospy.Publisher("background", String, queue_size=10)

    def sleep(self, data):
        global l
        if data.data == 0:
            l.append(data.data)
            if len(l) == 50: #20 seconds
                self.blink.publish(0)
                time.sleep(0.5)
                self.eyes.publish("sleep")
        elif data.data == 1 and len(l)>50:
            l=[]
            self.blink.publish(1)




if __name__ == '__main__':
    rospy.init_node("routine")
    node = Node()
    rospy.loginfo("idmind_elmo: sleep")
    rospy.spin()