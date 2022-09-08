#! /usr/bin/env python3


import time
import board
import neopixel
import colorsys

import rospy
from std_msgs.msg import ColorRGBA
from elmo.msg import Colors


LOOP_RATE = 5


class Node:

    def __init__(self):
        self.pixels = neopixel.NeoPixel(board.D18, 169, brightness=0.4, auto_write=False)
        rospy.Subscriber("leds/colors", Colors, self.on_colors)

    def on_colors(self, msg):
        for i in range(169):
            color = msg.colors[i] if len(msg.colors) > i else ColorRGBA()
            color_r = max(0, min(255, int(color.r)))
            color_g = max(0, min(255, int(color.g)))
            color_b = max(0, min(255, int(color.b)))
            self.pixels[i] = (color_r, color_g, color_b)

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            self.pixels.show()
        self.pixels.fill((0, 0, 0))
        self.pixels.show()


if __name__ == '__main__':
  rospy.init_node("leds")
  NODE = Node()
  rospy.loginfo(rospy.get_name() + ": running")
  NODE.run()
