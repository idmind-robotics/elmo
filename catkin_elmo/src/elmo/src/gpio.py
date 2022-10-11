#! /usr/bin/env python3


import rospy
import RPi.GPIO as GPIO
import os


RATE = 60


class Node:
    def __init__(self):
        self.shutdown_pin = rospy.get_param("gpio/shutdown_pin")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.shutdown_pin, GPIO.IN)
    
    def run(self):
        rate = rospy.Rate(RATE)
        pressed_counter = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if GPIO.input(self.shutdown_pin):
                if pressed_counter < RATE:
                    pressed_counter += 1
                else:
                    os.system("sudo shutdown -h now")
            else:
                pressed_counter = 0


if __name__ == '__main__':
    rospy.init_node("gpio")
    NODE = Node()
    rospy.loginfo("gpio: running")
    NODE.run()
