#! /usr/bin/env python3


import rospy
import robot as r


TIMEOUT = 5


class Node:
    def __init__(self):
        self.server_api = r.Server()
        self.onboard_api = r.Onboard()
        self.sound_api = r.Sound()
        self.leds_api = r.Leds()
        self.behaviours_api = r.Behaviours()

    def run(self):
        rospy.sleep(TIMEOUT)
        image = rospy.get_param("startup/image", "normal.png")
        if image:
            self.onboard_api.set_image(self.server_api.url_for_image(image))
        icon = rospy.get_param("startup/icon", "elmo_idm.png")
        if icon:
            self.leds_api.load_from_url(self.server_api.url_for_icon(icon))
        behaviours = rospy.get_param("startup/behaviours", [])
        for b in behaviours:
            self.behaviours_api.enable_behaviour(b)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("startup")
    NODE = Node()
    rospy.loginfo("startup: running")
    NODE.run()
