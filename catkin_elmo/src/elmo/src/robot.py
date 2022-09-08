#! /usr/bin/env python


import json
import colorsys
from PIL import Image
import requests
from io import BytesIO
import rospy
from elmo.msg import Colors, TouchEvent, PanTilt as PanTiltMsg
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String


class Leds:
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    def __init__(self):
        self.pub = rospy.Publisher("leds/colors", Colors, queue_size=10)

    def load_from_url(self, url):
        msg = Colors()
        response = requests.get(url)
        img = BytesIO(response.content)
        image = Image.open(img)
        for row in range(13):
            for col in range(13):
                # color = image.getpixel((col, 12 - row))
                color = image.getpixel((12 - col, row))
                msg.colors.append(ColorRGBA(r=color[0], g=color[1], b=color[2], a=0))
        self.pub.publish(msg)

    def indexed_rainbow(self):
        msg = Colors()
        for i in range(169):
            hue = 1.0 / 169 * i
            color_rgb = [int(c*255) for c in colorsys.hsv_to_rgb(hue, 1, 1)]
            msg.colors.append(ColorRGBA(
                color_rgb[0],
                color_rgb[1],
                color_rgb[2],
                0
            ))
        self.pub.publish(msg)

    def all(self, color):
        msg = Colors()
        for _ in range(169):
            msg.colors.append(ColorRGBA(
                color[0],
                color[1],
                color[2],
                0
            ))
        self.pub.publish(msg)

    def clear(self):
        msg = Colors()
        self.pub.publish(msg)


class Server:

    def get_icon_list(self):
        icon_list = requests.get("http://elmo:8000/icons").json()
        return icon_list

    def url_for_icon(self, icon_name):
        return "http://elmo:8000/icons/" + icon_name

    def get_image_list(self):
        image_list = requests.get("http://elmo:8000/images").json()
        return image_list

    def url_for_image(self, image_name):
        return "http://elmo:8000/images/" + image_name


class Onboard:

    def __init__(self):
        self.state = {

        }
        self.command_pub = rospy.Publisher("onboard/command", String, queue_size=10)
        rospy.Subscriber("onboard/state", String, self.__on_state)
    
    def __on_state(self, msg):
        self.state = json.loads(msg.data)

    def get_state(self):
        return self.state

    def set_image(self, image_name):
        command = {
            "image": image_name,
            "text": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def set_text(self, text):
        command = {
            "image": None,
            "text": text
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)


class Touch:
    CHEST = 0
    HEAD_N = 1
    HEAD_S = 2
    HEAD_E = 3
    HEAD_W = 4

    def __init__(self):
        self.touch_status = [
            False,
            False,
            False,
            False,
            False,
        ]
        self.callbacks = [
            [],
            [],
            [],
            [],
            [],
        ]
        rospy.Subscriber("touch", TouchEvent, self.__on_touch_event)

    def __on_touch_event(self, msg):
        for i in [Touch.CHEST, Touch.HEAD_N, Touch.HEAD_S, Touch.HEAD_E, Touch.HEAD_W]:
            if msg.sensor[i] and not self.touch_status[i]:
                for cb in self.callbacks[i]:
                    cb()
            self.touch_status[i] = msg.sensor[i]

    def on_touch(self, idx, cb):
        self.callbacks[idx].append(cb)

    def on_touch_head(self, cb):
        for idx in [Touch.HEAD_N, Touch.HEAD_S, Touch.HEAD_E, Touch.HEAD_W]:
            self.callbacks[idx].append(cb)

    def on_touch_chest(self, cb):
        self.callbacks[Touch.CHEST].append(cb)


class PanTilt:
    def __init__(self):
        self.status = PanTiltMsg()
        def on_status(msg):
            self.status = msg
        rospy.Subscriber("pan_tilt/status", PanTiltMsg, on_status)
        self.command_pub = rospy.Publisher("pan_tilt/command", PanTiltMsg, queue_size=1)
            
    def get_limits(self):
        return {
            "min_pan_angle": rospy.get_param("pan_tilt/min_pan_angle"),
            "max_pan_angle": rospy.get_param("pan_tilt/max_pan_angle"),
            "min_tilt_angle": rospy.get_param("pan_tilt/min_tilt_angle"),
            "max_tilt_angle": rospy.get_param("pan_tilt/max_tilt_angle"),
            "min_playtime": rospy.get_param("pan_tilt/min_playtime"),
            "max_playtime": rospy.get_param("pan_tilt/max_playtime"),
        }

    def get_status(self):
        return {
            "pan_enabled": self.status.pan_torque,
            "pan_angle": self.status.pan_angle,
            "pan_temperature": self.status.pan_temperature,
            "tilt_enabled": self.status.tilt_torque,
            "tilt_angle": self.status.tilt_angle,
            "tilt_temperature": self.status.tilt_temperature
        }

    def enable(self, pan, tilt):
        msg = PanTiltMsg()
        msg.pan_torque = pan
        msg.tilt_torque = tilt
        self.command_pub.publish(msg)
        rospy.sleep(0.5)
    
    def set_angles(self, pan=0, tilt=0, relative=False, playtime=0):
        msg = PanTiltMsg()
        if relative:
            msg.pan_torque = True if pan != 0 else self.status.pan_torque
            msg.pan_angle = self.status.pan_angle + pan
            msg.tilt_torque = True if tilt != 0 else self.status.tilt_torque
            msg.tilt_angle = self.status.tilt_angle + tilt
            msg.playtime = playtime
        else:
            msg.pan_torque = True if pan != 0 else self.status.pan_torque
            msg.pan_angle = pan
            msg.tilt_torque = True if tilt != 0 else self.status.tilt_torque
            msg.tilt_angle = tilt
            msg.playtime = playtime
        self.command_pub.publish(msg)

    def reset_angles(self):
        limits = self.get_limits()
        pan = (limits["max_pan_angle"] + limits["min_pan_angle"]) / 2.0
        tilt = (limits["max_tilt_angle"] + limits["min_tilt_angle"]) / 2.0
        self.set_angles(pan=pan, tilt=tilt, relative=False)


def test():
    rospy.init_node("test_node")
