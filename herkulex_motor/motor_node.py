#! /usr/bin/env python2.7

import herkulex
from herkulex import servo

import rospy
from std_msgs.msg import Int16


class Node(object):
    def __init__(self):
        herkulex.connect("/dev/ttyAMA0", 115200)
        herkulex.clear_errors()
        rospy.Subscriber("id_servo", Int16, self.servo_id)
        rospy.Subscriber("position", Int16, self.run)
        self.s= servo(3) #vertical
        self.l=servo(4) #horizontal
        self.s.torque_on()
        self.l.torque_on()

    def servo_id(self, data):
        choice = int(data.data)
        if choice == 4:
            self.motor = self.s 
        elif choice == 3:
            self.motor = self.l


    def run(self, data):
        position = int(data.data)
        self.motor.set_servo_position(position, 50, 0x00)


    def cleanup(self):
        rospy.loginfo("idmind_herkulex: cleaning up")
        herkulex.clear_errors()
        rospy.sleep(1.0)
        self.s.torque_off()
        self.l.torque_off()
        herkulex.close()


if __name__ == '__main__':
    rospy.init_node("herkulex")
    node = Node()
    rospy.on_shutdown(node.cleanup)
    rospy.loginfo("idmind_herkulex: running")
    rospy.spin()