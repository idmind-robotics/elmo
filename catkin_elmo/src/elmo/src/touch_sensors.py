#!/usr/bin/env python3

import time
import board
import busio
import adafruit_mpr121


import rospy
from elmo.msg import TouchEvent
from std_msgs.msg import UInt32MultiArray


LOOP_RATE = 10


class Node:
    def __init__(self):
        while not rospy.is_shutdown() and not rospy.get_param("robot_setup"):
            rospy.sleep(0.1)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mpr121 = adafruit_mpr121.MPR121(i2c)
        self.pub = rospy.Publisher('touch_old', TouchEvent, queue_size=10)
        self.pub_raw = rospy.Publisher('touch/raw', UInt32MultiArray, queue_size=10)

    @property
    def head_threshold(self):
        return rospy.get_param("touch/head_threshold")

    @property
    def chest_threshold(self):
        return rospy.get_param("touch/chest_threshold")

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = TouchEvent()
            msg.sensor[0] = bool(self.mpr121.filtered_data(0) < self.chest_threshold)
            msg.sensor[1] = bool(self.mpr121.filtered_data(1) < self.head_threshold)
            msg.sensor[2] = bool(self.mpr121.filtered_data(2) < self.head_threshold)
            msg.sensor[3] = bool(self.mpr121.filtered_data(3) < self.head_threshold)
            msg.sensor[4] = bool(self.mpr121.filtered_data(4) < self.head_threshold)
            for i in range(5):
                if msg.sensor[i]:
                    print("TOUCH " + str(i))
            print(" ")
            self.pub.publish(msg)
            msg_raw = UInt32MultiArray(data=[0, 0, 0, 0, 0])
            msg_raw.data[0] = self.mpr121.filtered_data(0)
            msg_raw.data[1] = self.mpr121.filtered_data(1)
            msg_raw.data[2] = self.mpr121.filtered_data(2)
            msg_raw.data[3] = self.mpr121.filtered_data(3)
            msg_raw.data[4] = self.mpr121.filtered_data(4)
            self.pub_raw.publish(msg_raw)


if __name__ == '__main__':
    rospy.init_node("touch_sensors")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
