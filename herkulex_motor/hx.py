#! /usr/bin/env python2.7


import herkulex
from herkulex import servo

import rospy
from std_msgs.msg import String

min4 = 170
max4 = 424
min3 = 80
max3 = 120


class Node(object):
    def __init__(self):
        herkulex.connect("/dev/ttyAMA0", 115200)
        herkulex.clear_errors()
        rospy.Subscriber("id", String, self.id)
        rospy.Subscriber("position", String, self.run)


    def id(self, data):
        self.servoid = int(data.data)
        if self.servoid == 3:
            self.s = servo(3)
        elif self.servoid == 4:
            self.s = servo(4)
        else:
            print("no servo with this id")         

    def run(self, data):
        self.s.torque_on()
        position = int(data.data)
        if self.servoid == 4:
            if position > min4 and position <max4: 
                self.s.set_servo_position(position, 200, 0x00)
            else:
                print("position out of limit")
        elif self.servoid == 3:
            if position > min3 and position <max3: 
                self.s.set_servo_position(position, 50, 0x00)
            else:
                print("position out of limit")
        else:
            print("no servo with this id")  
  

    def cleanup(self):
        rospy.loginfo("idmind_herkulex: cleaning up")
        herkulex.clear_errors()
        rospy.sleep(1.0)
        self.s.torque_off()
        herkulex.close()
            

if __name__ == '__main__':
    rospy.init_node("herkulex")
    node = Node()
    rospy.on_shutdown(node.cleanup)
    rospy.loginfo("idmind_herkulex: running")
    rospy.spin()