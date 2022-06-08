#!/usr/bin/env python3

import time
import board
import busio
import adafruit_mpr121


import rospy
from elmo.msg import TouchEvent


LOOP_RATE = 20


class Node:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mpr121 = adafruit_mpr121.MPR121(i2c)
        self.pub = rospy.Publisher('touch', TouchEvent, queue_size=10)

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = TouchEvent()
            msg.sensor[0] = bool(self.mpr121[0].value)
            msg.sensor[1] = bool(self.mpr121[1].value)
            msg.sensor[2] = bool(self.mpr121[2].value)
            msg.sensor[3] = bool(self.mpr121[3].value)
            msg.sensor[4] = bool(self.mpr121[4].value)
            self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("touch_sensors")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
