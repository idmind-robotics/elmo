#! /usr/bin/env python3



import os
import math
import json

import rospy
from elmo.msg import PanTilt

import robot as r
import logger


LOOP_RATE = 10
ANGLE_EQUALITY_THRESHOLD = 5
IDLE_ERROR_TIMEOUT = 5
ANGLE_DIFF_MOVING_THRESHOLD = 2
STATUS_UPDATE_TIMEOUT = 10


class Node:

    def __init__(self):
        self.logger = logger.Logger()
        self.last_status_at = rospy.Time.now()
        self.status = PanTilt()
        rospy.Subscriber("pan_tilt/status", PanTilt, self.on_pan_tilt_status)

    def on_pan_tilt_status(self, msg):
        self.status = msg
        self.last_status_at = rospy.Time.now()

    def reset_pan_tilt(self):
        self.logger.error("resetting pan tilt")
        os.system("rosnode kill /pan_tilt")
        rospy.sleep(5)
        os.system("rosrun elmo pan_tilt.py &")
        rospy.sleep(5)
        self.logger.info("pan tilt reset")

    def run(self):
        last_pan = 0
        last_tilt = 0
        is_moving = False
        should_be_moving = False
        last_move_time = rospy.Time.now()
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            rate.sleep()
            if (rospy.Time.now() - self.last_status_at).to_sec() > STATUS_UPDATE_TIMEOUT:
                self.logger.error("pan tilt status not updated in %s seconds" % STATUS_UPDATE_TIMEOUT)
                self.reset_pan_tilt()
                self.last_status_at = rospy.Time.now()
                continue
            status = self.status
            pan_angle = status.pan_angle
            tilt_angle = status.tilt_angle
            pan_angle_ref = status.pan_angle_ref
            tilt_angle_ref = status.tilt_angle_ref
            pan_diff = math.fabs(pan_angle - pan_angle_ref)
            tilt_diff = math.fabs(tilt_angle - tilt_angle_ref)
            pan_delta = math.fabs(pan_angle - last_pan)
            tilt_delta = math.fabs(tilt_angle - last_tilt)
            should_be_moving = pan_diff > 5 or tilt_diff > 5
            is_moving = pan_delta > 0.1 or tilt_delta > 0.1
            if is_moving:
                last_move_time = rospy.Time.now()
            last_pan = pan_angle
            last_tilt = tilt_angle
            if should_be_moving and (rospy.Time.now() - last_move_time).to_sec() > IDLE_ERROR_TIMEOUT:
                self.reset_pan_tilt()
            # if should_be_moving:
            #     if is_moving:
            #         self.logger.info("is moving")
            #     else:
            #         self.logger.warn("should be moving")




if __name__ == "__main__":
    rospy.init_node("pan_tilt_watchdog")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
