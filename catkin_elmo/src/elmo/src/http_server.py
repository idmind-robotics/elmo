#! /usr/bin/env python


import os
import json
from flask import Flask, send_from_directory, request, jsonify
from flask_cors import CORS
import threading
import rospy
from std_msgs.msg import String


"""

This module serves the contents inside the folder specified by the ros parameter <static_folder> via HTTP

"""


app = Flask(__name__, static_url_path='')
CORS(app)


ONBOARD_COMMAND = {
    "text": "",
    "image": "http://elmo:8000/images/sunglasses.png",
}


ONBOARD_STATE = {

}


@app.route("/")
def index():
    print("serving: index")
    return send_from_directory(rospy.get_param("http_server/static"), "index.html")
    

@app.route("/api/onboard/command")
def onboard_command():

    return jsonify(ONBOARD_COMMAND)


@app.route("/api/onboard/state", methods=["POST"])
def onboard_state():
    state = request.json
    for k in state:
        ONBOARD_STATE[k] = state[k]
    onboard_state_description = json.dumps(ONBOARD_STATE)
    state_pub.publish(onboard_state_description)
    return jsonify({})


@app.route("/icons")
def icons():
    print("serving: icons")
    icon_list = os.listdir(rospy.get_param("http_server/static") + "/icons")
    return jsonify(icon_list)


@app.route("/images")
def images():
    print("serving: images")
    image_list = os.listdir(rospy.get_param("http_server/static") + "/images")
    return jsonify(image_list)

@app.route("/sounds")
def sounds():
    print("serving: sounds")
    sounds_list = os.listdir(rospy.get_param("http_server/static") + "/sounds")
    return jsonify(sounds_list)

@app.route("/speech")
def speech():
    print("serving: speech")
    speech_list = os.listdir(rospy.get_param("http_server/static") + "/speech")
    return jsonify(speech_list)


if __name__ == "__main__":
    rospy.init_node("http_server")
    server_port = rospy.get_param("http_server/port")
    server_thread = threading.Thread(target=lambda: app.run(debug=False, port=server_port, host="0.0.0.0"))
    server_thread.setDaemon(True)
    server_thread.start()
    rospy.loginfo("server running on port " + str(server_port))
    # ROS API
    def on_command(msg):
        command = json.loads(msg.data)
        for k in command:
            ONBOARD_COMMAND[k] = command[k]
    rospy.Subscriber("/onboard/command", String, on_command)
    state_pub = rospy.Publisher("/onboard/state", String, queue_size=10)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
    print("server shutting down")