#! /usr/bin/env python


import os
import subprocess
import netifaces
import time
import rospy


class Node:

    def __init__(self):
        while not rospy.is_shutdown() and not rospy.get_param("robot_setup"):
            rospy.sleep(0.1)
        self.close_hotspot()
        time.sleep(5.0)
        if rospy.has_param("wifi"):
            creds = rospy.get_param("wifi")
            connected_to_wifi = self.connect(creds["ssid"], creds["password"])
            time.sleep(1.0)
            if connected_to_wifi:
                print("connected to %s" % creds["ssid"])
            elif not self.check_ip():
                time.sleep(1.0)
                self.open_hotspot()
        else:
            self.open_hotspot()

    def connect(self, ssid, password):
        try:
            command = "nmcli dev wifi connect %s password %s" % (ssid, password)
            rospy.loginfo(command)
            out = subprocess.check_output(command.split(" "))
            rospy.loginfo(out)
            return "successfully activated" in out
        except:
            return False

    def check_ip(self):
        try:
            ip = netifaces.ifaddresses("wlan0")[netifaces.AF_INET][0]["addr"]
            rospy.loginfo("checking ip: %s" % ip)
            return ip != ""
        except Exception:
            return False

    def open_hotspot(self):
        try:
            print("opening hotspot")
            command = "nmcli con up Elmo"
            os.system(command)
            return True
        except:
            return False

    def close_hotspot(self):
        try:
            print("closing hotspot")
            command = "nmcli con down Elmo"
            os.system(command)
            return True
        except:
            return False


if __name__ == '__main__':
    rospy.init_node("wifi")
    NODE = Node()
    rospy.loginfo("wifi: running")
    rospy.spin()
