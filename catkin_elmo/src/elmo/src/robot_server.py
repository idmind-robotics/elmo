#! /usr/bin/env python3


import threading
import socket
import json
from flask import Flask, jsonify, request
import robot as r


SERVER_PORT = 8000


app = Flask(
    __name__,
    static_url_path="",
)


class Robot:

    pan_tilt_api = r.PanTilt()
    touch_api = r.Touch()
    server_api = r.Server()
    leds_api = r.Leds()
    onboard_api = r.Onboard()


    def __init__(self):
        self.pan = 0
        self.tilt = 0
        self.pan_min = 0
        self.pan_max = 80
        self.tilt_min = -90
        self.tilt_max = 90
        self.pan_torque = False
        self.tilt_torque = False
        self.pan_temperature = 0
        self.tilt_temperature = 0
        self.touch_chest = False
        self.touch_head_n = False
        self.touch_head_s = False
        self.touch_head_e = False
        self.touch_head_w = False
        self.behaviour_test_motors = False
        self.behaviour_test_leds = False
        self.speech_list = [
            "speech 01",
            "speech 02",
            "speech 03",
        ]
        self.sound_list = [
            "sound 01",
            "sound 02",
            "sound 03",
        ]
        self.face_list = [
            "face 01",
            "face 02"
        ]
        self.icon_list = [
            "icon 01",
            "icon 01 (red)"
        ]
        self.volume = 0
        self.video_port = "8080"
        self.video_path = "/stream?topic=/usb_cam/image_raw"

    def update(self):
        # motors
        status = self.pan_tilt_api.get_status()
        self.pan = status["pan_angle"]
        self.pan_torque = status["pan_enabled"]
        self.pan_temperature = status["pan_temperature"]
        self.tilt = status["tilt_angle"]
        self.tilt_torque = status["tilt_enabled"]
        self.tilt_temperature = status["tilt_temperature"]
        limits = self.pan_tilt_api.get_limits()
        self.pan_min = limits["min_pan_angle"]
        self.pan_max = limits["max_pan_angle"]
        self.tilt_min = limits["min_tilt_angle"]
        self.tilt_max = limits["max_tilt_angle"]
        # touch
        self.touch_chest = self.touch_api.touch_status[r.Touch.CHEST]
        self.touch_head_n = self.touch_api.touch_status[r.Touch.HEAD_N]
        self.touch_head_s = self.touch_api.touch_status[r.Touch.HEAD_S]
        self.touch_head_e = self.touch_api.touch_status[r.Touch.HEAD_E]
        self.touch_head_w = self.touch_api.touch_status[r.Touch.HEAD_W]
        # behaviours
        # server
        self.speech_list = self.server_api.get_speech_list()
        self.sound_list = self.server_api.get_sound_list()
        self.face_list = self.server_api.get_image_list()
        self.icon_list = self.server_api.get_icon_list()
        # sound


    def enable_test_motors(self, control):
        return True, "OK"

    def enable_test_leds(self, control):
        return True, "OK"

    def set_pan_torque(self, control):
        self.pan_tilt_api.enable(control, self.tilt_torque)
        return True, "OK"

    def set_pan(self, angle):
        self.pan_tilt_api.set_angles(pan=angle)
        return True, "OK"

    def set_tilt_torque(self, control):
        self.pan_tilt_api.enable(self.pan_torque, control)
        return True, "OK"

    def set_tilt(self, angle):
        self.pan_tilt_api.set_angles(tilt=angle)
        return True, "OK"

    def update_motor_limits(self, pan_min, pan_max, tilt_min, tilt_max):
        self.pan_tilt_api.set_limits(
            pan_min,
            pan_max,
            tilt_min,
            tilt_max
        )
        return True, "OK"

    def play_sound(self, name):
        return True, "OK"

    def play_speech(self, name):
        return True, "OK"

    def pause_audio(self):
        return True, "OK"

    def set_volume(self, v):
        return True, "OK"

    def update_leds(self, colors):
        self.leds_api.set_colors(colors)
        return True, "OK"

    def update_leds_icon(self, name):
        url = self.server_api.url_for_icon(name)
        self.leds.load_from_url(url)
        return True, "OK"

    def set_screen(self, image=None, text=None, url=None):
        if image is not None:
            self.onboard_api.set_image(image)
        elif text is not None:
            self.onboard_api.set_text(text)
        return True, "OK"

    def shutdown(self):
        return True, "OK"



robot = Robot()


@app.route("/status")
def status():
    robot.update()
    return jsonify(robot.__dict__)


@app.route("/command", methods=["POST"])
def command():
    try:
        req = request.json
        print(req)
        success = True
        message = "OK"
        op = req["op"]
        if op == "enable_behaviour":
            name = req["name"]
            control = req["control"]
            if name == "test_motors":
                success, message = robot.enable_test_motors(control)
            if name == "test_leds":
                success, message = robot.enable_test_leds(control)
            return jsonify({ "success": True, "message": "OK" })
        elif op == "set_pan_torque":
            control = req["control"]
            success, message = robot.set_pan_torque(control)
        elif op == "set_pan":
            angle = req["angle"]
            success, message = robot.set_pan(angle)
        elif op == "set_tilt_torque":
            control = req["control"]
            success, message = robot.set_tilt_torque(control)
        elif op == "set_tilt":
            angle = req["angle"]
            success, message = robot.set_tilt(angle)
        elif op == "update_motor_limits":
            pan_min = req["pan_min"]
            pan_max = req["pan_max"]
            tilt_min = req["tilt_min"]
            tilt_max = req["tilt_max"]
            success, message = robot.update_motor_limits(pan_min, pan_max, tilt_min, tilt_max)
        elif op == "play_sound":
            name = req["name"]
            success, message = robot.play_sound(name)
        elif op == "play_speech":
            name = req["name"]
            success, message = robot.play_speech(name)
        elif op == "pause_audio":
            success, message = robot.pause_audio()
        elif op == "set_volume":
            volume = req["volume"]
            success, message = robot.set_volume(volume)
        elif op == "set_screen":
            image = req["image"]
            text = req["text"]
            url = req["url"]
            success, message = robot.set_screen(image, text, url)
        elif op == "update_leds":
            colors = req["colors"]
            success, message = robot.update_leds(colors)
        elif op == "update_leds_icon":
            name = req["name"]
            success, message = robot.update_leds_icon(name)
        elif op == "reboot":
            success, message = robot.reboot()
        elif op == "shutdown":
            success, message = robot.shutdown()
        else:
            return jsonify({ "success": False, "message": "%s is not a recognized operation" % op })
        return jsonify({ "success": success, "message": message })
    except Exception as e:
        return jsonify({ "success": False, "message": str(e) })


def quick_connect():
    udp_ip = "0.0.0.0"
    udp_port = 5000
    response = b"iamarobot;elmo;ELMO OG;8001"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if b"ruarobot" in data:
                sock.sendto(response, addr)
        except socket.timeout:
            pass


if __name__ == '__main__':
    udp_server_thread = threading.Thread(target=quick_connect)
    udp_server_thread.setDaemon(True)
    udp_server_thread.start()
    server_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=8001))
    server_thread.setDaemon(True)
    server_thread.start()
    input("running")
