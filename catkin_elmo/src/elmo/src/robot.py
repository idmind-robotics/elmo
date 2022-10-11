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
from std_srvs.srv import Trigger
import os
import threading


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

    def set_colors(self, colors):
        msg = Colors()
        for row in range(13):
            for col in range(13):
                r, g, b = colors[row*13+12-col]
                msg.colors.append(ColorRGBA(r=r, g=g, b=b, a=0))
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

    def __init__(self):
        self.port = rospy.get_param("http_server/port")
        self.image_address = "/images"
        self.icon_address = "/icons"
        self.speech_address = "/speech"
        self.sound_address = "/sounds"

    def get_icon_list(self):
        url = "http://elmo:" + str(self.port) + self.icon_address
        icon_list = requests.get(url).json()
        return icon_list

    def url_for_icon(self, icon_name):
        url = "http://elmo:" + str(self.port) + self.icon_address
        return url + "/" + icon_name

    def get_image_list(self):
        url = "http://elmo:" + str(self.port) + self.image_address
        image_list = requests.get(url).json()
        return image_list

    def url_for_image(self, image_name):
        url = "http://elmo:" + str(self.port) + self.image_address
        return url + "/" + image_name

    def get_sound_list(self):
        url = "http://elmo:" + str(self.port) + self.sound_address
        sound_list = requests.get(url).json()
        return sound_list

    def url_for_sound(self, sound_name):
        url = "http://elmo:" + str(self.port) + self.sound_address
        return url + "/" + sound_name

    def get_speech_list(self):
        url = "http://elmo:" + str(self.port) + self.speech_address
        speech_list = requests.get(url).json()
        return speech_list

    def url_for_speech(self, speech_name):
        url = "http://elmo:" + str(self.port) + self.speech_address
        return url + "/" + speech_name


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

    def set_camera_feed(self):
        command = {
            "image": "http://elmo:8081/",
            "text": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def get_default_image(self):
        return rospy.get_param("onboard/default_image")


class Touch:
    CHEST = 0
    HEAD_W = 1
    HEAD_N = 2
    HEAD_E = 3
    HEAD_S = 4

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

    def get_touch_head_threshold(self):
        return rospy.get_param("touch/head_threshold")

    def get_touch_chest_threshold(self):
        return rospy.get_param("touch/chest_threshold")
    
    def set_touch_head_threshold(self, threshold):
        rospy.set_param("touch/head_threshold", threshold)
    
    def set_touch_chest_threshold(self, threshold):
        rospy.set_param("touch/chest_threshold", threshold)


class PanTilt:
    def __init__(self):
        self.status = PanTiltMsg()
        def on_status(msg):
            self.status = msg
        rospy.Subscriber("pan_tilt/status", PanTiltMsg, on_status)
        self.command_pub = rospy.Publisher("pan_tilt/command", PanTiltMsg, queue_size=1)
        self.reload_params_srv = rospy.ServiceProxy("pan_tilt/reload_params", Trigger)
            
    def get_limits(self):
        return {
            "min_pan_angle": rospy.get_param("pan_tilt/min_pan_angle"),
            "max_pan_angle": rospy.get_param("pan_tilt/max_pan_angle"),
            "min_tilt_angle": rospy.get_param("pan_tilt/min_tilt_angle"),
            "max_tilt_angle": rospy.get_param("pan_tilt/max_tilt_angle"),
            "min_playtime": rospy.get_param("pan_tilt/min_playtime"),
            "max_playtime": rospy.get_param("pan_tilt/max_playtime"),
        }

    def set_limits(self, min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle):
        rospy.set_param("pan_tilt/min_pan_angle", min_pan_angle)
        rospy.set_param("pan_tilt/max_pan_angle", max_pan_angle)
        rospy.set_param("pan_tilt/min_tilt_angle", min_tilt_angle)
        rospy.set_param("pan_tilt/max_tilt_angle", max_tilt_angle)
        self.reload_params_srv()

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


class Sound:

    def play_sound_from_url(self, url):
        os.system("curl %s | aplay" % url)

    def set_volume(self, v):
        print("SET VOLUME")
        print(v)
        os.system("amixer sset 'Master' {}%".format(v))


class Power:

    def shutdown(self):
        def do_shutdown():
            os.system("/usr/sbin/shutdown -h now")
        shutdown_thread = threading.Thread(target=do_shutdown)
        shutdown_thread.start()


class Behaviours:

    def __init__(self):
        self.enable_pub = rospy.Publisher("behaviour/enable", String, queue_size=1)
        self.disable_pub = rospy.Publisher("behaviour/disable", String, queue_size=1)

    def get_behaviour_list(self):
        behaviours = rospy.get_param("behaviours").keys()
        return behaviours

    def enable_behaviour(self, behaviour):
        print("enabling " + behaviour)
        self.enable_pub.publish(behaviour)

    def disable_behaviour(self, behaviour):
        print("disabling " + behaviour)
        self.disable_pub.publish(behaviour)

    def is_behaviour_enabled(self, behaviour):
        return rospy.get_param("behaviour/" + behaviour + "/enabled")

class Wifi:
    def __init__(self):
        self.credentials_file = rospy.get_param("wifi/credentials")

    def update_credentials(self, ssid, password):
        rospy.loginfo("updating wifi credentials")
        with open(self.credentials_file, "w") as fp:
            json.dump({
                "ssid": ssid,
                "password": password
            }, fp)
        rospy.logwarn("wifi credentials updated")


def test():
    rospy.init_node("test_node")
