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

    def on_shutdown():
        behaviour_api.disable_behaviour("test_motors")
        leds_api.clear()
    rospy.on_shutdown(on_shutdown)

    rospy.sleep(5.0)
    behaviour_api.enable_behaviour("test_motors")
    url = server_api.url_for_sound("hd.wav")
    sound_api.play_sound_from_url(url)
    while not rospy.is_shutdown():
        url = server_api.url_for_icon("heartbeat.gif")
        leds_api.load_from_url(url)
    rospy.loginfo("done")




