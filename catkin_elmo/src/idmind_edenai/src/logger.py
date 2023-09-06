#! /usr/bin/env python3


import rospy
from std_msgs.msg import String


class Logger:

    def __init__(self):
        self.pub = rospy.Publisher("/ai/logging", String, queue_size=10)
        self.info_pub = rospy.Publisher("/ai/logging/info", String, queue_size=10)
        self.warn_pub = rospy.Publisher("/ai/logging/warn", String, queue_size=10)
        self.error_pub = rospy.Publisher("/ai/logging/error", String, queue_size=10)

    def info(self, msg):
        rospy.loginfo(msg)
        log = "[INFO] " + rospy.get_name() + ": " + msg
        self.info_pub.publish(log)
        self.pub.publish(log)

    def warn(self, msg):
        rospy.logwarn(msg)
        log = "[WARN] " + rospy.get_name() + ": " + msg
        self.warn_pub.publish(log)
        self.pub.publish(log)
    
    def error(self, msg):
        rospy.logerr(msg)
        log = "[ERROR] " + rospy.get_name() + ": " + msg
        self.error_pub.publish(log)
        self.pub.publish(log)

    