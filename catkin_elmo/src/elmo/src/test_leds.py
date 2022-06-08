#! /usr/bin/env python3


import robot as r
import rospy


def main():
    rospy.init_node("image_to_leds")
    leds = r.Leds()
    server = r.Server()
    icons = server.get_icon_list()
    while not rospy.is_shutdown():
        for icon in icons:
            rospy.sleep(3.0)
            icon_url = server.url_for_icon(icon)
            leds.load_from_url(icon_url)
    leds.clear()


if __name__ == '__main__':
    main()
