#! /usr/bin/env python3


import json
import rospy
from std_srvs.srv import Trigger


class Node:
    def __init__(self):
        self.config_file = rospy.get_param("config_file")
        try:
            with open(self.config_file) as fp:
                config = json.load(fp)
                for k in config:
                    rospy.set_param(k, config[k])
        except:
            self.export()
        rospy.set_param("robot_setup", True)
        rospy.Service("config/export", Trigger, self.on_export)

    def export(self):
        config = {}
        for k in [
            "robot_name",
            "touch/head_threshold",
            "touch/chest_threshold",
            "pan_tilt/pan_p",
            "pan_tilt/pan_d",
            "pan_tilt/min_pan_angle",
            "pan_tilt/max_pan_angle",
            "pan_tilt/tilt_p",
            "pan_tilt/tilt_d",
            "pan_tilt/min_tilt_angle",
            "pan_tilt/max_tilt_angle",
            "pan_tilt/max_tilt_angle",
            "wifi/ssid",
            "wifi/password",
            "startup/image",
            "startup/icon",
            "startup/behaviours"
        ]:
            if rospy.has_param(k):
                v = rospy.get_param(k)
                config[k] = v
        with open(self.config_file, "w") as fp:
            json.dump(config, fp, indent=2)

    def on_export(self, _):
        self.export()
        return True, "OK"


if __name__ == '__main__':
    rospy.init_node("config")
    NODE = Node()
    rospy.loginfo("config: running")
    rospy.spin()
