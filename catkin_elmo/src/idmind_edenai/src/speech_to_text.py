#! /usr/bin/env python3


import threading

import os
import rospy
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
import requests
import logger


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
        self.initial_energy = 0
        self.logger = logger.Logger()
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
        self.logger.info("Enabled")
        return True, "OK"

    def on_disable(self, _):
        self.enabled = False
        self.logger.info("Disabled")
        return True, "OK"

    def initialize(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        # initial noise calibration
        max_value = 0.0
        n_tries = 0
        duration = 30
        self.logger.info("Initialized")
        with self.microphone as source:
            for i in range(n_tries):
                self.recognizer.adjust_for_ambient_noise(source, duration=duration)
                if self.recognizer.energy_threshold > max_value:
                    max_value = self.recognizer.energy_threshold
                self.logger.info("Calibration %d/%d. Minimum energy: %f" % (i + 1, n_tries, self.recognizer.energy_threshold))
        self.recognizer.energy_threshold = max_value
        self.logger.info("Calibration done. Minimum energy: %f" % self.recognizer.energy_threshold)
        self.initial_energy = self.recognizer.energy_threshold

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            self.heartbeat_pub.publish()
            if not self.enabled:
                continue
            try:
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source, 1.0)
                if self.recognizer.energy_threshold < self.initial_energy:
                    self.logger.warn("Energy threshold too low. Resetting to initial value.")
                    self.recognizer.energy_threshold = self.initial_energy
                # self.recognizer.energy_threshold = 2000
                with self.microphone as source:
                    self.logger.warn("listening")
                    # self.recognizer.dynamic_energy_threshold = False
                    audio = self.recognizer.listen(source)
                    # check duration
                    duration = len(audio.frame_data) / (audio.sample_rate * audio.sample_width)
                    self.logger.warn("duration: %f" % duration)
                # self.logger.warn("recording")
                # with open("out.wav", "wb") as f:
                #     f.write(audio.get_wav_data())
                self.logger.warn("transcribing")
                # self.recognizer.dynamic_energy_threshold = True
                if self.language == "pt":
                    # value = self.recognizer.recognize_sphinx(audio, language="pt-PT")
                    # value = self.recognizer.recognize_vosk(audio, language="pt-PT")
                    value = self.recognizer.recognize_google(audio, language="pt-PT")
                else:
                    # value = self.recognizer.recognize_sphinx(audio)
                    # value = self.recognizer.recognize_vosk(audio)
                    value = self.recognizer.recognize_google(audio)
                self.logger.warn("recognized: %s" % value)
                self.pub.publish(value)
            except sr.UnknownValueError:
                if self.unrecognized is not None:
                    os.system("aplay %s" % self.unrecognized)
                # else:
                #     msg = UNRECOGNIZED
                #     self.logger.warn(msg)
                #     self.pub.publish(msg)
            except sr.RequestError as e:
                msg = "Unable to request results from Google Speech Recognition Service: %s" % str(e)
                self.logger.error(msg)
            except sr.WaitTimeoutError:
                pass
            except KeyboardInterrupt:
                self.logger.error("canceled")
                break
            except Exception as e:
                self.logger.error(str(e))
                continue

if __name__ == '__main__':
    rospy.init_node("speech_to_text", disable_signals=True)
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
