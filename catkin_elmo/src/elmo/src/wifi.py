#! /usr/bin/env python


import os
import subprocess
import netifaces
import time
import rospy
import json


class Node:

    def __init__(self):
        credentials_file = rospy.get_param("wifi/credentials")
        self.close_hotspot()
        time.sleep(2.0)
        try:
            rospy.loginfo("looking for wifi credentials in %s" % credentials_file)
            with open(credentials_file) as fp:
                creds = json.load(fp)
            connected_to_wifi = self.connect(creds["ssid"], creds["password"])
            time.sleep(1.0)
            if not connected_to_wifi and self.check_ip():
                time.sleep(1.0)
                self.open_hotspot()
        except:
            rospy.loginfo("no credentials found")
            self.open_hotspot()


    def connect(self, ssid, password):
        try:
            print("connecting to wifi")
            command = "nmcli dev wifi connect %s password \"%s\"" % (ssid, password)
            # os.system(command)
            print(command)
            out = subprocess.check_output(command.split(" ")).decode()
            return "successfully activated" in out
        except:
            return False

    def check_ip(self):
        try:
            print("checking if valid IP address")
            ip = netifaces.ifaddresses("wlan0")[netifaces.AF_INET][0]["addr"]
            print("ip: %s" % ip)
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
