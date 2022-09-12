#! /usr/bin/env python3


import socket
import netifaces
import requests
import threading


CONTEXT = {
    "scanning_robots": False
}

MAX_ERROR_COUNT = 10


def scan_robots(cb):
    def scan_robots_runnable():
        while CONTEXT["scanning_robots"]:
            interfaces = netifaces.interfaces()
            allips = []
            for i in interfaces:
                allips.append(netifaces.ifaddresses(i)[netifaces.AF_INET][0]["addr"])
            msg = b'ruarobot'
            for ip in allips:
                if "127.0.0" in ip:
                    continue
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
                    sock.settimeout(1)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    sock.bind((ip,0))
                    sock.sendto(msg, ("255.255.255.255", 5000))
                    data, address = sock.recvfrom(1024)
                    if b"iamarobot" in data:
                        header, robot_model, robot_name, server_port = data.decode("utf-8").split(";")
                        cb(robot_name, "http://%s:%s" % (address[0], server_port))
                    sock.close()
                except:
                    pass
    CONTEXT["scanning_robots"] = True
    t = threading.Thread(target=scan_robots_runnable)
    t.start()

def stop_scan():
    CONTEXT["scanning_robots"] = False


def connect(address):
    try:
        return True, "OK", Robot(address)
    except Exception as e:
        return False, e, None        


class Robot:

    def __init__(self, address):
        self.address = address
        ip, port = address.split("/")[2].split(":")
        self.ip = ip
        self.port = port
        self.error_count = 0

    def update_status(self):
        try:
            url = self.address + "/status"
            new_status = requests.get(url).json()
            for k in new_status:
                setattr(self, k, new_status[k])
            self.error_count = 0
        except Exception as e:
            if self.error_count < MAX_ERROR_COUNT:
                self.error_count += 1
            else:
                self.on_fatal_error()
            print(e)

    def send_command(self, command, **kwargs):
        try:
            url = self.address + "/command"
            kwargs["op"] = command
            res = requests.post(url, json=kwargs).json()
            if res["success"]:
                return True
            else:
                self.on_error(res["message"])
                return False
        except Exception as e:
            print(e)

    def on_error(self, message):
        print("Error: " + message)

    def on_fatal_error(self):
        print("Fatal error")
