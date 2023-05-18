#! /usr/bin/env python


import numpy as np

import rospy
from std_msgs.msg import UInt32MultiArray
from elmo.msg import TouchEvent


def calculate_bollinger_bands(numbers, window_size, num_std):
    # Calculate rolling mean and standard deviation
    rolling_mean = np.convolve(numbers, np.ones(window_size)/window_size, mode='valid')
    rolling_std = np.sqrt(np.convolve(np.square(numbers - rolling_mean), np.ones(window_size)/window_size, mode='valid'))

    # Calculate upper and lower bands
    upper_band = rolling_mean + num_std * rolling_std
    lower_band = rolling_mean - num_std * rolling_std

    return upper_band, rolling_mean, lower_band


WINDOW_SIZE = 100
STD_DEV = 5.0


class Node:

    def __init__(self):
        self.windows = []
        self.limits = []
        rospy.Subscriber("touch/raw", UInt32MultiArray, self.on_touch_values)
        self.pub = rospy.Publisher("touch", TouchEvent, queue_size=10)
        print("calibrating")
        self.waiting_for_data = True
        while self.waiting_for_data:
            rospy.sleep(1.0)
        print("calculating limits")
        for window in self.windows:
            upper_band, rolling_mean, lower_band = calculate_bollinger_bands(window, len(window), STD_DEV)
            self.limits.append({
                "upper": upper_band,
                "lower": lower_band
            })
        print("calibration successful")

    def on_touch_values(self, msg):
        for idx, v in enumerate(msg.data):
            if idx >= len(self.windows):
                self.windows.append([])
            self.windows[idx].append(v)
            self.windows[idx] = self.windows[idx][-WINDOW_SIZE:]
        if len(self.windows[0]) == WINDOW_SIZE:
            self.waiting_for_data = False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            print("---")
            msg = TouchEvent()
            for idx, window in enumerate(self.windows):
                values = window[-3:]
                touch = all([value < self.limits[idx]["lower"] for value in values])
                msg.sensor[idx] = touch
                if touch:
                    print("TOUCH: %d" % idx)
                else:
                    print("no touch: %d" % idx)
            self.pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node("touch_calibrator")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
