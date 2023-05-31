#! /usr/bin/env python3


import rospy
import robot as r


TIMEOUT = 5
LOOP_RATE = 10
TOUCH_COUNTER_THRESHOLD = 3


class Node:
    def __init__(self):
        self.touch_api = r.Touch()
        self.server_api = r.Server()
        self.onboard_api = r.Onboard()
        self.sound_api = r.Sound()
        self.leds_api = r.Leds()
        self.last_animation_at = 0
        self.touch_counter = 0

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.touch_api.head_touch_detected():
                if self.touch_counter < TOUCH_COUNTER_THRESHOLD:
                    self.touch_counter += 1
                if self.touch_counter == TOUCH_COUNTER_THRESHOLD:
                    if self.last_animation_at == 0 or (rospy.Time.now() - self.last_animation_at).secs > TIMEOUT:
                        print("animation")
                        self.last_animation_at = rospy.Time.now()
                        self.onboard_api.set_image(self.server_api.url_for_image("love.png"))
                        self.sound_api.play_sound_from_url(self.server_api.url_for_sound("love.wav"))
                        self.leds_api.load_from_url(self.server_api.url_for_icon("heartbeat.gif"))
                        rospy.sleep(3.0)
                        self.onboard_api.set_image(self.server_api.url_for_image("normal.png"))
                        self.leds_api.load_from_url(self.server_api.url_for_icon("elmo_idm.png"))
            else:
                self.touch_counter = 0
            print(rospy.get_name() + ": touch counter: " + str(self.touch_counter))


if __name__ == '__main__':
    rospy.init_node("blush")
    NODE = Node()
    rospy.loginfo("blush: running")
    NODE.run()
