#! /usr/bin/env python3


from translate import Translator

import rospy
from std_msgs.msg import String

import logger


class Node:
    def __init__(self):
        self.logger = logger.Logger()
        self.from_lang = rospy.get_param("~from_lang")
        self.to_lang = rospy.get_param("~to_lang")
        self.translator = Translator(from_lang=self.from_lang, to_lang=self.to_lang)
        rospy.Subscriber("input", String, self.on_translate)
        self.pub = rospy.Publisher("output", String, queue_size=10)

    def on_translate(self, msg):
        data = msg.data
        self.logger.info("Translating '%s' to '%s'..." % (data, self.to_lang))
        translated = self.translator.translate(data)
        self.logger.info("Translated '%s' to '%s'" % (data, translated))
        self.pub.publish(translated)


if __name__ == "__main__":
    rospy.init_node("translate")
    node = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    rospy.spin()
