#! /usr/bin/env python3


import os
import json
from flask import Flask, send_from_directory, request, jsonify
from flask_cors import CORS
import threading
import rospy
from std_msgs.msg import String
from werkzeug.utils import secure_filename


"""

This module serves the contents inside the folder specified by the ros parameter <static_folder> via HTTP

"""


app = Flask(__name__, static_url_path='')
CORS(app)


ONBOARD_COMMAND = {
    "text": "",
    "image": "http://elmo:8000/images/normal.png",
    "video": "",
    "url": "",

}


ONBOARD_STATE = {

}


@app.route("/")
def index():
    return send_from_directory(rospy.get_param("http_server/static"), "index.html")
    

@app.route("/api/onboard/command")
def onboard_command():
    res = jsonify(ONBOARD_COMMAND)
    ONBOARD_COMMAND["text"] = ""
    ONBOARD_COMMAND["image"] = ""
    ONBOARD_COMMAND["video"] = ""
    ONBOARD_COMMAND["url"] = ""
    return res


@app.route("/api/onboard/state", methods=["POST"])
def onboard_state():
    state = request.json
    for k in state:
        ONBOARD_STATE[k] = state[k]
    onboard_state_description = json.dumps(ONBOARD_STATE)
    state_pub.publish(onboard_state_description)
    return jsonify({})


@app.route("/icons", methods=["GET", "POST"])
def icons():
    if request.method == "GET":
        icon_list = os.listdir(rospy.get_param("http_server/static") + "/icons")
        return jsonify(icon_list)
    elif request.method == "POST":
        file = request.files['file']
        filename = secure_filename(file.filename)
        path = rospy.get_param("http_server/static") + "/icons/"
        file.save(path + filename)
        return jsonify("OK")


@app.route("/icons/<name>", methods=["DELETE"])
def delete_icon(name):
    if request.method == "DELETE":
        full_name = rospy.get_param("http_server/static") + "/icons/" + name
        print("deleting " + full_name)
        os.remove(full_name)
        return jsonify("OK")


@app.route("/images", methods=["GET", "POST"])
def images():
    if request.method == "GET":
        image_list = os.listdir(rospy.get_param("http_server/static") + "/images")
        return jsonify(image_list)
    elif request.method == "POST":
        file = request.files['file']
        filename = secure_filename(file.filename)
        path = rospy.get_param("http_server/static") + "/images/"
        print("uploading " + path + filename) 
        file.save(path + filename)
        return jsonify("OK")


@app.route("/images/<name>", methods=["DELETE"])
def delete_image(name):
    if request.method == "DELETE":
        full_name = rospy.get_param("http_server/static") + "/images/" + name
        print("deleting " + full_name)
        os.remove(full_name)
        return jsonify("OK")


@app.route("/sounds", methods=["GET", "POST"])
def sounds():
    if request.method == "GET":        
        sounds_list = os.listdir(rospy.get_param("http_server/static") + "/sounds")
        return jsonify(sounds_list)
    elif request.method == "POST":
        file = request.files['file']
        filename = secure_filename(file.filename)
        path = rospy.get_param("http_server/static") + "/sounds/"
        file.save(path + filename)
        return jsonify("OK")


@app.route("/sounds/<name>", methods=["DELETE"])
def delete_sound(name):
    if request.method == "DELETE":
        full_name = rospy.get_param("http_server/static") + "/sounds/" + name
        print("deleting " + full_name)
        os.remove(full_name)
        return jsonify("OK")


@app.route("/videos", methods=["GET", "POST"])
def videos():
    if request.method == "GET":        
        videos_list = os.listdir(rospy.get_param("http_server/static") + "/videos")
        return jsonify(videos_list)
    elif request.method == "POST":
        file = request.files['file']
        filename = secure_filename(file.filename)
        path = rospy.get_param("http_server/static") + "/videos/"
        file.save(path + filename)
        return jsonify("OK")


@app.route("/videos/<name>", methods=["DELETE"])
def delete_video(name):
    if request.method == "DELETE":
        full_name = rospy.get_param("http_server/static") + "/videos/" + name
        print("deleting " + full_name)
        os.remove(full_name)
        return jsonify("OK")


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