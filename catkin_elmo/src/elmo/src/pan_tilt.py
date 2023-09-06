#! /usr/bin/env python3


import math
import herkulex as hx

import rospy
from utils import linterpol

from elmo.msg import PanTilt
from std_srvs.srv import Trigger



LOOP_RATE = 10
DEBUG = False
MAX_ERRORS_BEFORE_RESET = 5


class Node:
    def __init__(self):
        while not rospy.is_shutdown() and not rospy.get_param("robot_setup"):
            rospy.sleep(0.1)
        self.connect()
        self.n_errors = 0
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
        rospy.Subscriber("pan_tilt/command_raw", PanTilt, self.on_command_raw)
        rospy.Service("pan_tilt/recalibrate_pid", Trigger, self.on_recalibrate_pid)

    def connect(self):
        rospy.loginfo(rospy.get_name() + ": Connecting to herkulex buffer...")
        hx.connect("/dev/ttyS0", 115200)
        hx.clear_errors()
        pan_id = rospy.get_param("/pan_tilt/pan_id")
        tilt_id = rospy.get_param("/pan_tilt/tilt_id")
        rospy.loginfo(rospy.get_name() + ": Connecting to servos...")
        self.pan = hx.servo(pan_id)
        self.tilt = hx.servo(tilt_id)
        rospy.loginfo(rospy.get_name() + ": Calibrating servos...")
        self.calibrate_pid()

    @property
    def pan_pid_p(self):
        return rospy.get_param("pan_tilt/pan_p")

    @property
    def pan_pid_d(self):
        return rospy.get_param("pan_tilt/pan_d")

    @property
    def max_pan_angle(self):
        return rospy.get_param("pan_tilt/max_pan_angle")

    @property
    def min_pan_angle(self):
        return rospy.get_param("pan_tilt/min_pan_angle")

    @property
    def tilt_pid_p(self):
        return rospy.get_param("pan_tilt/tilt_p")

    @property
    def tilt_pid_d(self):
        return rospy.get_param("pan_tilt/tilt_d")

    @property
    def max_tilt_angle(self):
        return rospy.get_param("pan_tilt/max_tilt_angle")

    @property
    def min_tilt_angle(self):
        return rospy.get_param("pan_tilt/min_tilt_angle")

    @property
    def min_playtime(self):
        return rospy.get_param("pan_tilt/min_playtime")

    @property
    def max_playtime(self):
        return rospy.get_param("pan_tilt/max_playtime")

    def calibrate_pid(self):
        rospy.sleep(0.5)
        self.pan.set_position_p(self.pan_pid_p)
        rospy.sleep(0.5)
        self.pan.set_position_d(self.pan_pid_d)
        rospy.sleep(0.5)
        self.tilt.set_position_p(self.tilt_pid_p)
        rospy.sleep(0.5)
        self.tilt.set_position_d(self.tilt_pid_d)
        rospy.sleep(0.5)

    def on_recalibrate_pid(self, _):
        self.calibrate_pid()
        return True, "OK"

    def on_command_raw(self, msg):
        # get requested parameters
        self.command_pan_torque = msg.pan_torque
        self.command_tilt_torque = msg.tilt_torque
        self.command_pan_angle = msg.pan_angle
        self.command_tilt_angle = msg.tilt_angle
        self.command_playtime = msg.playtime
        # limit range
        self.command_pan_angle = max(self.min_pan_angle, min(self.command_pan_angle, self.max_pan_angle))
        self.command_tilt_angle = max(self.min_tilt_angle, min(self.command_tilt_angle, self.max_tilt_angle))

    def on_command(self, msg):
        # get requested parameters
        self.command_pan_torque = msg.pan_torque
        self.command_tilt_torque = msg.tilt_torque
        self.command_pan_angle = msg.pan_angle
        self.command_tilt_angle = msg.tilt_angle
        self.command_playtime = msg.playtime * 100 if msg.playtime > 0 else self.min_playtime
        # limit range
        self.command_pan_angle = max(self.min_pan_angle, min(self.command_pan_angle, self.max_pan_angle))
        self.command_tilt_angle = max(self.min_tilt_angle, min(self.command_tilt_angle, self.max_tilt_angle))
        # calculate min playtime
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
                # rate.sleep()
                # hx.clear_errors()
                rate.sleep()
                # update angle status
                self.status_pan_angle = self.pan.get_servo_angle()
                self.status_tilt_angle = self.tilt.get_servo_angle()
                # update temperature status
                # self.status_pan_temperature = self.pan.get_servo_temperature()
                # self.status_tilt_temperature = self.tilt.get_servo_temperature()
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
                    else:
                        self.pan.torque_off()
                    self.status_pan_torque = self.command_pan_torque
                    # continue
                elif self.command_pan_angle != self.status_pan_angle_ref:
                    self.pan.set_servo_angle(self.command_pan_angle, int(self.command_playtime), 0)
                    # print("pan: %.2f %d" % (self.command_pan_angle, self.command_playtime))
                    # self.pan.set_servo_angle(self.command_pan_angle, 100, 0)
                    self.status_pan_angle_ref = self.command_pan_angle
                    # continue
                rospy.sleep(1.0 / LOOP_RATE / 2.0)  # wait for pan to finish
                # tilt
                if self.command_tilt_torque != self.status_tilt_torque:
                    if self.command_tilt_torque:
                        self.tilt.torque_on()
                    else:
                        self.tilt.torque_off()
                    self.status_tilt_torque = self.command_tilt_torque
                    # continue
                elif self.command_tilt_angle != self.status_tilt_angle_ref:
                    self.tilt.set_servo_angle(self.command_tilt_angle, int(self.command_playtime), 0)
                    # print("tilt: %.2f %d" % (self.command_tilt_angle, self.command_playtime))
                    # self.tilt.set_servo_angle(self.command_tilt_angle, 100, 0)
                    self.status_tilt_angle_ref = self.command_tilt_angle
                    # continue
                self.n_errors = 0
            except Exception as e:
                try:
                    self.n_errors += 1
                    self.hx.clear_errors()
                    if self.n_errors > MAX_ERRORS_BEFORE_RESET:
                        self.n_errors = 0
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
                        rospy.logerr(rospy.get_name() + ": " + str(e))
                        rospy.logerr("attempting to recover")
                        rospy.sleep(1.0)
                        rospy.loginfo("closing connection")
                        hx.close()
                        rospy.sleep(1.0)
                        rospy.loginfo("opening connection")
                        self.connect()
                except:
                    pass

        rospy.logwarn("disabling torque")
        self.pan.torque_off()
        self.tilt.torque_off()


if __name__ == '__main__':
    rospy.init_node("pan_tilt_node")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()
