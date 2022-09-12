#! /usr/bin/env python3


import rospy
import robot as r
from std_msgs.msg import String



class Node:

    def __init__(self):
        self.leds_api = r.Leds()
        self.server_api = r.Server()
        self.enabled = False
        rospy.set_param("behaviour/test_leds/enabled", self.enabled)
        rospy.Subscriber("behaviour/enable", String, self.on_enable)
        rospy.Subscriber("behaviour/disable", String, self.on_disable)

    def on_enable(self, msg):
        if msg.data != "test_leds" or self.enabled:
            return
        print("enabling test leds")
        self.enabled = True
        rospy.set_param("behaviour/test_leds/enabled", self.enabled)

    def on_disable(self, msg):
        if msg.data != "test_leds" or not self.enabled:
            return
        print("disabling test leds")
        self.leds_api.clear()
        self.enabled = False
        rospy.set_param("behaviour/test_leds/enabled", self.enabled)

    def run(self):
        rate = rospy.Rate(10)
        icon_idx = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if self.enabled:
                icons = self.server_api.get_icon_list()
                if not len(icons):
                    continue
                if icon_idx >= len(icons):
                    icon_idx = 0
                icon = icons[icon_idx]
                icon_url = self.server_api.url_for_icon(icon)
                self.leds_api.load_from_url(icon_url)
                icon_idx += 1
                rospy.sleep(3.0)


if __name__ == '__main__':
    rospy.init_node("test_leds")
    NODE = Node()
    rospy.loginfo("test_leds: running")
    NODE.run()
