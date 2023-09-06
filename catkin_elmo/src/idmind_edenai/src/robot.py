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
from std_msgs.msg import Int32
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
        # gif
        if ".gif" in url:
            print("loading gif")
            response = requests.get(url)
            img = BytesIO(response.content)
            image = Image.open(img)
            # create image buffer
            messages = []
            try:
                while 1:
                    image.seek(image.tell() + 1)
                    msg = Colors()
                    for row in range(13):
                        for col in range(13):
                            im = image.convert("RGB")
                            color = im.getpixel((12 - col, row))
                            msg.colors.append(ColorRGBA(r=color[0], g=color[1], b=color[2], a=0))
                    messages.append(msg)
            except EOFError:
                msg = Colors()
                messages.append(msg)
            # schedule the publishing of the messages
            time_between_frames = image.info["duration"] / 1000.0
            for i in range(len(messages)):
                threading.Timer(time_between_frames * i, self.pub.publish, args=[messages[i]]).start()
        else:
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
        self.computer_name = "elmo"
        self.image_address = "/images"
        self.icon_address = "/icons"
        self.sound_address = "/sounds"
        self.video_address = "/videos"

    def get_icon_list(self):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.icon_address
        icon_list = requests.get(url).json()
        return icon_list

    def url_for_icon(self, icon_name):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.icon_address
        return url + "/" + icon_name

    def get_image_list(self):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.image_address
        image_list = requests.get(url).json()
        return image_list

    def url_for_image(self, image_name):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.image_address
        return url + "/" + image_name

    def get_sound_list(self):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.sound_address
        sound_list = requests.get(url).json()
        return sound_list

    def url_for_sound(self, sound_name):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.sound_address
        return url + "/" + sound_name

    def get_video_list(self):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.video_address
        video_list = requests.get(url).json()
        return video_list

    def url_for_video(self, video_name):
        url = "http://" + self.computer_name + ":" + str(self.port) + self.video_address
        return url + "/" + video_name


class Onboard:

    def __init__(self):
        self.user_request_callbacks = {}
        self.state = {

        }
        self.command_pub = rospy.Publisher("onboard/command", String, queue_size=10)
        rospy.Subscriber("onboard/state", String, self.__on_state)
        rospy.Subscriber("onboard/user_request", String, self.on_user_request)
    
    def __on_state(self, msg):
        self.state = json.loads(msg.data)

    def user_request_cb(self, request, callback):
        self.user_request_callbacks[request] = callback

    def on_user_request(self, msg):
        request = msg.data
        if request in self.user_request_callbacks:
            self.user_request_callbacks[request]()

    def get_state(self):
        return self.state

    def set_image(self, image_name):
        command = {
            "image": image_name,
            "text": None,
            "video": None,
            "url": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def set_text(self, text):
        command = {
            "image": None,
            "text": text,
            "video": None,
            "url": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def set_camera_feed(self):
        command = {
            "image": "http://elmo:8081/",
            "text": None,
            "video": None,
            "url": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def play_video(self, video_name, start_time=0.0, end_time=0.0):
        command = {
            "image": None,
            "text": None,
            "video": {
                "video_name": video_name,
                "start_time": start_time,
                "end_time": end_time
            },
            "url": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def set_url(self, url):
        command = {
            "image": None,
            "text": None,
            "video": None,
            "url": url
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)

    def get_default_image(self):
        return rospy.get_param("onboard/default_image")

    def is_playing_video(self):
        return self.state["video"] is not None and self.state["video"] != ""

    def reset(self):
        command = {
            "image": None,
            "text": None,
            "video": None,
            "url": None
        }
        command_description = json.dumps(command)
        self.command_pub.publish(command_description)


class Touch:
    CHEST = 0
    HEAD_N = 4 # 2
    HEAD_W = 3 # 1
    HEAD_S = 1 # 4
    HEAD_E = 2 # 3
    TOUCH_BUFFER_SIZE = 4

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
        self.touch_buffers = {
            "chest": 0,
            "head_n": 0,
            "head_w": 0,
            "head_s": 0,
            "head_e": 0,
        }
        rospy.Subscriber("touch", TouchEvent, self.__on_touch_event)

    def __on_touch_event(self, msg):
        for i in [Touch.CHEST, Touch.HEAD_N, Touch.HEAD_S, Touch.HEAD_E, Touch.HEAD_W]:
            if msg.sensor[i] and not self.touch_status[i]:
                for cb in self.callbacks[i]:
                    cb()
            self.touch_status[i] = msg.sensor[i]
        if msg.sensor[Touch.CHEST]:
            if self.touch_buffers["chest"] < Touch.TOUCH_BUFFER_SIZE:
                self.touch_buffers["chest"] += 1
        else:
            self.touch_buffers["chest"] = 0
        if msg.sensor[Touch.HEAD_N]:
            if self.touch_buffers["head_n"] < Touch.TOUCH_BUFFER_SIZE:
                self.touch_buffers["head_n"] += 1
        else:
            self.touch_buffers["head_n"] = 0
        if msg.sensor[Touch.HEAD_W]:
            if self.touch_buffers["head_w"] < Touch.TOUCH_BUFFER_SIZE:
                self.touch_buffers["head_w"] += 1
        else:
            self.touch_buffers["head_w"] = 0
        if msg.sensor[Touch.HEAD_S]:
            if self.touch_buffers["head_s"] < Touch.TOUCH_BUFFER_SIZE:
                self.touch_buffers["head_s"] += 1
        else:
            self.touch_buffers["head_s"] = 0
        if msg.sensor[Touch.HEAD_E]:
            if self.touch_buffers["head_e"] < Touch.TOUCH_BUFFER_SIZE:
                self.touch_buffers["head_e"] += 1
        else:
            self.touch_buffers["head_e"] = 0

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

    def head_touch_detected(self):
        return any((
            self.touch_buffers["head_n"] == Touch.TOUCH_BUFFER_SIZE,
            self.touch_buffers["head_w"] == Touch.TOUCH_BUFFER_SIZE,
            self.touch_buffers["head_s"] == Touch.TOUCH_BUFFER_SIZE,
            self.touch_buffers["head_e"] == Touch.TOUCH_BUFFER_SIZE
        ))

    def chest_touch_detected(self):
        return self.touch_buffers["chest"] == Touch.TOUCH_BUFFER_SIZE




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

    def set_limits(self, min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle):
        rospy.set_param("pan_tilt/min_pan_angle", min_pan_angle)
        rospy.set_param("pan_tilt/max_pan_angle", max_pan_angle)
        rospy.set_param("pan_tilt/min_tilt_angle", min_tilt_angle)
        rospy.set_param("pan_tilt/max_tilt_angle", max_tilt_angle)

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

    def __init__(self):
        self.url_pub = rospy.Publisher("speakers/url", String, queue_size=10)
        self.volume_pub = rospy.Publisher("speakers/volume", Int32, queue_size=10)

    def play_sound_from_url(self, url):
        self.url_pub.publish(url)

    def set_volume(self, v):
        self.volume_pub.publish(v)


class Power:
    def __init__(self):
        self.will_shutdown = False

    def shutdown(self):
        if not self.will_shutdown:
            self.will_shutdown = True
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
        self.export_srv = rospy.ServiceProxy("config/export", Trigger)

    def update_credentials(self, ssid, password):
        rospy.loginfo("updating wifi credentials")
        rospy.set_param("wifi/ssid", ssid)
        rospy.set_param("wifi/password", password)
        self.export_srv()
        rospy.logwarn("wifi credentials updated")


def test():
    rospy.init_node("test_node")
