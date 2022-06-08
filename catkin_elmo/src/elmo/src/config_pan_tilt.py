#! /usr/bin/env python


import json
import herkulex as hx

import rospy

from dynamic_reconfigure.server import Server
from elmo.cfg import PanTiltConfig


class Node:
    def __init__(self):
        hx.connect("/dev/ttyS0", 115200)
        hx.clear_errors()
        self.is_first_config = True
        self.pan_id = 0
        self.pan_torque = False
        self.pan_pid_p = 0
        self.pan_pid_d = 0
        self.pan_pid_i = 0
        self.pan_position = 0
        self.pan_playtime = 0
        self.tilt_id = 0
        self.tilt_torque = False
        self.tilt_pid_p = 0
        self.tilt_pid_d = 0
        self.tilt_pid_i = 0
        self.tilt_position = 0
        self.tilt_playtime = 0
        self.pan = None
        self.tilt = None
        self.dynamic_server = Server(PanTiltConfig, callback=self.on_config)


    def on_config(self, config, _):
        if self.is_first_config:
            self.is_first_config = False
            return self.first_config(config)
        return self.update_config(config)

    def first_config(self, config):
        rospy.logwarn("first config")
        rospy.loginfo(json.dumps(config, indent=2))
        # connect to pan and get values from servo
        self.pan_id = config.pan_id
        self.pan_torque = config.pan_torque
        self.pan = hx.servo(self.pan_id)
        self.pan_pid_p = self.pan.get_position_p()
        self.pan_pid_d = self.pan.get_position_d()
        self.pan_pid_i = self.pan.get_position_i()
        self.pan_position = self.pan.get_servo_position()
        # connect to tilt and get values from servo
        self.tilt_id = config.tilt_id
        self.tilt_torque = config.tilt_torque
        self.tilt = hx.servo(self.tilt_id)
        self.tilt_pid_p = self.tilt.get_position_p()
        self.tilt_pid_d = self.tilt.get_position_d()
        self.tilt_pid_i = self.tilt.get_position_i()
        self.tilt_position = self.tilt.get_servo_position()
        # correct config
        config.pan_id = self.pan_id
        config.pan_torque = self.pan_torque
        config.pan_pid_p = self.pan_pid_p
        config.pan_pid_d = self.pan_pid_d
        config.pan_pid_i = self.pan_pid_i
        config.pan_position = self.pan_position
        config.pan_playtime = self.pan_playtime
        config.tilt_id = self.tilt_id
        config.tilt_torque = self.tilt_torque
        config.tilt_pid_p = self.tilt_pid_p
        config.tilt_pid_d = self.tilt_pid_d
        config.tilt_pid_i = self.tilt_pid_i
        config.tilt_position = self.tilt_position
        config.tilt_playtime = self.tilt_playtime

        return config

    def update_config(self, config):
        rospy.logwarn("new config")
        # update pan servo
        if config.pan_id != self.pan_id:
            self.pan = hx.servo(self.pan_id)
        if config.pan_torque != self.pan_torque:
            if config.pan_torque:
                self.pan.torque_on()
            else:
                self.pan.torque_off()
        if config.pan_pid_p != self.pan_pid_p:
            self.pan.set_position_p(config.pan_pid_p)
        if config.pan_pid_d != self.pan_pid_d:
            self.pan.set_position_d(config.pan_pid_d)
        if config.pan_pid_i != self.pan_pid_i:
            self.pan.set_position_i(config.pan_pid_i)
        if config.pan_position != self.pan_position:
            self.pan.set_servo_position(config.pan_position, config.pan_playtime, 0)
        # update internal pan config
        self.pan_id = config.pan_id
        self.pan_torque = config.pan_torque
        self.pan_pid_p = config.pan_pid_p
        self.pan_pid_d = config.pan_pid_d
        self.pan_pid_i = config.pan_pid_i
        self.pan_position = config.pan_position
        self.pan_playtime = config.pan_playtime
        # update tilt servo
        if config.tilt_id != self.tilt_id:
            rospy.loginfo("connecting")
            self.tilt = hx.servo(self.tilt_id)
        if config.tilt_torque != self.tilt_torque:
            if config.tilt_torque:
                rospy.loginfo("torque ON")
                self.tilt.torque_on()
            else:
                rospy.loginfo("torque OFF")
                self.tilt.torque_off()
        if config.tilt_pid_p != self.tilt_pid_p:
            rospy.loginfo("calibrating PID")
            self.tilt.set_position_p(config.tilt_pid_p)
        if config.tilt_pid_d != self.tilt_pid_d:
            rospy.loginfo("calibrating PID")
            self.tilt.set_position_d(config.tilt_pid_d)
        if config.tilt_pid_i != self.tilt_pid_i:
            rospy.loginfo("calibrating PID")
            self.tilt.set_position_i(config.tilt_pid_i)
        if config.tilt_position != self.tilt_position:
            rospy.loginfo("moving")
            self.tilt.set_servo_position(config.tilt_position, config.tilt_playtime, 0)
        # update internal tilt config
        self.tilt_id = config.tilt_id
        self.tilt_torque = config.tilt_torque
        self.tilt_pid_p = config.tilt_pid_p
        self.tilt_pid_d = config.tilt_pid_d
        self.tilt_pid_i = config.tilt_pid_i
        self.tilt_position = config.tilt_position
        self.tilt_playtime = config.tilt_playtime

        return config

    def run(self):
        rospy.spin()
        rospy.logwarn("disabling torque")
        self.pan.torque_off()
        self.tilt.torque_off()


if __name__ == '__main__':
    rospy.init_node("pan_tilt_node")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    NODE.run()