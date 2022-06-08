#! /usr/bin/env python3


import math
import herkulex as hx

import rospy
from utils import linterpol

from elmo.msg import PanTilt



LOOP_RATE = 10
DEBUG = False


class Node:
    def __init__(self):
        hx.connect("/dev/ttyS0", 115200)
        hx.clear_errors()
        pan_id = rospy.get_param("pan_tilt/pan_id")
        pan_pid_p = rospy.get_param("pan_tilt/pan_p")
        pan_pid_d = rospy.get_param("pan_tilt/pan_d")
        tilt_id = rospy.get_param("pan_tilt/tilt_id")
        tilt_pid_p = rospy.get_param("pan_tilt/tilt_p")
        tilt_pid_d = rospy.get_param("pan_tilt/tilt_d")
        self.max_pan_angle = rospy.get_param("pan_tilt/max_pan_angle")
        self.min_pan_angle = rospy.get_param("pan_tilt/min_pan_angle")
        self.max_tilt_angle = rospy.get_param("pan_tilt/max_tilt_angle")
        self.min_tilt_angle = rospy.get_param("pan_tilt/min_tilt_angle")
        self.min_playtime = rospy.get_param("pan_tilt/min_playtime")
        self.max_playtime = rospy.get_param("pan_tilt/max_playtime")
        self.pan = hx.servo(pan_id)
        self.tilt = hx.servo(tilt_id)
        rospy.sleep(0.5)
        self.pan.set_position_p(pan_pid_p)
        rospy.sleep(0.5)
        self.pan.set_position_d(pan_pid_d)
        rospy.sleep(0.5)
        self.tilt.set_position_p(tilt_pid_p)
        rospy.sleep(0.5)
        self.tilt.set_position_d(tilt_pid_d)
        rospy.sleep(0.5)
        self.status_pan_torque = False
        self.status_pan_angle = self.pan.get_servo_angle()
        self.status_pan_angle_ref = self.status_pan_angle
        self.status_pan_temperature = self.pan.get_servo_temperature()
        self.status_tilt_torque = False
        self.status_tilt_angle = self.tilt.get_servo_angle()
        self.status_tilt_angle_ref = self.status_tilt_angle
        self.status_tilt_temperature = self.tilt.get_servo_temperature()
        self.command_pan_torque = False
        self.command_pan_angle = self.status_pan_angle
        self.command_tilt_torque = False
        self.command_tilt_angle = self.status_tilt_angle
        self.command_playtime = 0
        self.status_pub = rospy.Publisher("pan_tilt/status", PanTilt, queue_size=10)
        rospy.Subscriber("pan_tilt/command", PanTilt, self.on_command)

        if DEBUG:
            def set_pan(angle, playtime, _):
                pass
            def set_tilt(angle, playtime, _):
                pass
            self.pan.set_servo_angle = set_pan
            self.tilt.set_servo_angle = set_tilt


    def on_command(self, msg):
        # get requested parameters
        self.command_pan_torque = msg.pan_torque
        self.command_tilt_torque = msg.tilt_torque
        self.command_pan_angle = msg.pan_angle
        self.command_tilt_angle = msg.tilt_angle
        self.command_playtime = msg.playtime * 100
        # ignore 0 values for angle
        if self.command_pan_angle == 0:
            self.command_pan_angle = self.status_pan_angle_ref
        if self.command_tilt_angle == 0:
            self.command_tilt_angle = self.status_tilt_angle_ref
        # limit range
        self.command_pan_angle = max(self.min_pan_angle, min(self.command_pan_angle, self.max_pan_angle))
        self.command_tilt_angle = max(self.min_tilt_angle, min(self.command_tilt_angle, self.max_tilt_angle))
        # calculate min playtime
        if self.command_pan_angle != 0 or self.command_tilt_angle != 0:
            motion_range_pan = math.fabs(self.command_pan_angle - self.status_pan_angle)
            max_range_pan = self.max_pan_angle - self.min_pan_angle
            motion_range_pan_normalized = linterpol(motion_range_pan, 0, max_range_pan, 0.0, 1.0)
            motion_range_tilt = math.fabs(self.command_tilt_angle - self.status_tilt_angle)
            max_range_tilt = self.max_tilt_angle - self.min_tilt_angle
            motion_range_tilt_normalized = linterpol(motion_range_tilt, 0, max_range_tilt, 0.0, 1.0)
            # use largest motion for min playtime calculation
            if motion_range_pan_normalized > motion_range_tilt_normalized:
                min_playtime = linterpol(motion_range_pan, 0, self.max_pan_angle - self.min_pan_angle, self.min_playtime, self.max_playtime)
            else:
                min_playtime = linterpol(motion_range_tilt, 0, self.max_tilt_angle - self.min_tilt_angle, self.min_playtime, self.max_playtime)
            if self.command_playtime < min_playtime:
                self.command_playtime = min_playtime
            # limit upper playtime bound
            if self.command_playtime > self.max_playtime:
                self.command_playtime = self.max_playtime

    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            try:
                hx.clear_errors()
                rate.sleep()
                # update angle status
                self.status_pan_angle = self.pan.get_servo_angle()
                self.status_tilt_angle = self.tilt.get_servo_angle()
                # update temperature status
                self.status_pan_temperature = self.pan.get_servo_temperature()
                self.status_tilt_temperature = self.tilt.get_servo_temperature()
                # publish status
                status = PanTilt()
                status.pan_torque = self.status_pan_torque
                status.pan_angle = self.status_pan_angle
                status.pan_angle_ref = self.status_pan_angle_ref
                status.pan_temperature = self.status_pan_temperature
                status.tilt_torque = self.status_tilt_torque
                status.tilt_angle = self.status_tilt_angle
                status.tilt_angle_ref = self.status_tilt_angle_ref
                status.tilt_temperature = self.status_tilt_temperature
                self.status_pub.publish(status)
                # pan
                if self.command_pan_torque != self.status_pan_torque:
                    if self.command_pan_torque:
                        self.pan.torque_on()
                        rospy.logwarn("pan: ON")
                    else:
                        self.pan.torque_off()
                        rospy.logwarn("pan: OFF")
                    self.status_pan_torque = self.command_pan_torque
                    continue
                elif self.command_pan_angle != self.status_pan_angle_ref:
                    self.pan.set_servo_angle(self.command_pan_angle, int(self.command_playtime), 0)
                    rospy.loginfo("pan: %.2f\t| %d", self.command_pan_angle, self.command_playtime)
                    self.status_pan_angle_ref = self.command_pan_angle
                    continue
                # tilt
                if self.command_tilt_torque != self.status_tilt_torque:
                    if self.command_tilt_torque:
                        self.tilt.torque_on()
                        rospy.logwarn("tilt: ON")
                    else:
                        self.tilt.torque_off()
                        rospy.logwarn("tilt: OFF")
                    self.status_tilt_torque = self.command_tilt_torque
                    continue
                elif self.command_tilt_angle != self.status_tilt_angle_ref:
                    self.tilt.set_servo_angle(self.command_tilt_angle, int(self.command_playtime), 0)
                    rospy.loginfo("tilt: %.2f\t| %d", self.command_tilt_angle, self.command_playtime)
                    self.status_tilt_angle_ref = self.command_tilt_angle
                    continue
            except Exception as e:
                rospy.logerr(rospy.get_name() + str(e))

        rospy.logwarn("disabling torque")
        self.pan.torque_off()
        self.tilt.torque_off()


if __name__ == '__main__':
    rospy.init_node("pan_tilt_node")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
