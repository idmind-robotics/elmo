#! /usr/bin/env python3



import robot as r
import rospy



if __name__ == '__main__':
    rospy.init_node("start_routine")
    rospy.loginfo("start routine")

    server_api = r.Server()
    leds_api = r.Leds()
    onboard_api = r.Onboard()
    sound_api = r.Sound()
    behaviour_api = r.Behaviours()

    rospy.sleep(5.0)
    behaviour_api.enable_behaviour("test_motors")
    url = server_api.url_for_image("sleep.png")
    onboard_api.set_image(url)
    url = server_api.url_for_icon("heartbeat.gif")
    leds_api.load_from_url(url)
    url = server_api.url_for_image("normal.png")
    onboard_api.set_image(url)
    url = server_api.url_for_icon("heartbeat.gif")
    leds_api.load_from_url(url)
    behaviour_api.disable_behaviour("test_motors")
    rospy.loginfo("done")




