#! /usr/bin/env python


"""

This Node subscribes to speech on the topic /ai/speech_to_text/output.

If the speech detected contains all the KEYWORDS, it shows a video on the screen.

If the robot is already showing a video, it ignores the speech.

"""


import robot as r
import rospy
from std_msgs.msg import String



class Node:

    def __init__(self):
        self.keywords = rospy.get_param("~keywords", [
            "call",
            "friend",
            "polo"
        ])
        self.video = rospy.get_param("~video", "call.mp4")
        self.audio = rospy.get_param("~audio", "call.wav")
        self.onboard = r.Onboard()
        self.server = r.Server()
        self.sound = r.Sound()
        rospy.Subscriber("/ai/speech_to_text/output", String, self.on_speech)

    def on_speech(self, msg):
        if self.onboard.is_playing_video():
            return
        text = msg.data.lower()
        # check if any keywords are in the text
        if any([keyword in text for keyword in self.keywords]):
            video_url = self.server.url_for_video(self.video)
            audio_url = self.server.url_for_sound(self.audio)
            self.onboard.play_video(video_url)
            self.sound.play_sound_from_url(audio_url)


if __name__ == '__main__':
    rospy.init_node('call_friend')
    node = Node()
    rospy.loginfo(rospy.get_name() + ": running.")
    rospy.spin()
