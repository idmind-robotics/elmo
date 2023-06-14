#! /usr/bin/env python3


import os
import rospy
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
import requests

import speech_recognition as sr


"""

In order to install this module, a couple of dependencies may be necessary.

$ python3 -m pip install SpeechRecognition
$ python3 -m pip install pyaudio
$ sudo apt-get install portaudio19-dev python-pyaudio flac


"""


UNRECOGNIZED = "I'm sorry, I didn't understand that."


class Node:

    def __init__(self):
        self.heartbeat_pub = rospy.Publisher(rospy.get_name() + "/heartbeat", Empty, queue_size=10)
        self.enabled = rospy.get_param(rospy.get_name() + "/starts_enabled", True)
        self.language = rospy.get_param("language", "en")
        self.pub = rospy.Publisher(rospy.get_name() + "/output", String, queue_size=1)
        rospy.Service(rospy.get_name() + "/enable", Trigger, self.on_enable)
        rospy.Service(rospy.get_name() + "/disable", Trigger, self.on_disable)
        self.unrecognized = rospy.get_param("text_to_speech/unrecognized", None)
        self.recognizer = None
        self.microphone = None
        self.initialize()

    def on_enable(self, _):
        self.enabled = True
        return True, "OK"

    def on_disable(self, _):
        self.enabled = False
        return True, "OK"

    def initialize(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            self.heartbeat_pub.publish()
            if not self.enabled:
                continue
            try:
                # print("calibrating")
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source)
                # print("Calibration done. Minimum energy: %f" % self.recognizer.energy_threshold)
                self.recognizer.energy_threshold *= 2.0
                # self.recognizer.energy_threshold = 200
                print("Adjusting to %f" % self.recognizer.energy_threshold)
                with self.microphone as source:
                    print("listening")
                    audio = self.recognizer.listen(source)
                print("recording")
                with open("out.wav", "wb") as f:
                    f.write(audio.get_wav_data())
                print("transcribing")
                if self.language == "pt":
                    # value = self.recognizer.recognize_sphinx(audio, language="pt-PT")
                    # value = self.recognizer.recognize_vosk(audio, language="pt-PT")
                    value = self.recognizer.recognize_google(audio, language="pt-PT")
                else:
                    # value = self.recognizer.recognize_sphinx(audio)
                    # value = self.recognizer.recognize_vosk(audio)
                    value = self.recognizer.recognize_google(audio)
                print("recognized: %s" % value)
                self.pub.publish(value)
            except sr.UnknownValueError:
                if self.unrecognized is not None:
                    os.system("aplay %s" % self.unrecognized)
                else:
                    msg = UNRECOGNIZED
                    print(msg)
                    self.pub.publish(msg)
            except sr.RequestError as e:
                msg = "Unable to request results from Google Speech Recognition Service: %s" % str(e)
                print(msg)
            except sr.WaitTimeoutError:
                pass
            except KeyboardInterrupt:
                print("canceled")
                break
        print("exit")


if __name__ == '__main__':
    rospy.init_node("speech_to_text", disable_signals=True)
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
