#!/usr/bin/env python3

# prerequisites: as described in https://alphacephei.com/vosk/install and also python module `sounddevice` (simply run command `pip install sounddevice`)
# Example usage using Dutch (nl) recognition model: `python test_microphone.py -m nl`
# For more help run: `python test_microphone.py -h`

import argparse
import queue
import sys
import sounddevice as sd

from vosk import Model, KaldiRecognizer

q = queue.Queue()

device = None
filename = None
list_devices = False
model = None
samplerate = None

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

if list_devices:
    print(sd.query_devices())
    parser.exit(0)

import json
import rospy
from std_msgs.msg import String

try:

    rospy.init_node("speech_to_text", disable_signals=True)
    pub = rospy.Publisher(rospy.get_name() + "/output", String, queue_size=10)
    if samplerate is None:
        device_info = sd.query_devices(device, "input")
        # soundfile expects an int, sounddevice provides a float:
        samplerate = int(device_info["default_samplerate"])
        
    if model is None:
        model = Model(lang="en-us")
    else:
        model = Model(lang=model)

    if filename:
        dump_fn = open(filename, "wb")
    else:
        dump_fn = None
    with sd.RawInputStream(samplerate=samplerate, blocksize = 20000, device=device,
            dtype="int16", channels=1, callback=callback):

        rec = KaldiRecognizer(model, samplerate)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            print("before get")
            data = q.get()
            print("after get")
            if rec.AcceptWaveform(data):
                res = rec.Result()
                try:
                    res = json.loads(res)
                    print(res)
                    if "text" in res:
                        pub.publish(res["text"])
                except Exception as e:
                    print(e)
                    print(res)
                    print(type(res))
                # clear queue
                while not q.empty():
                    q.get()
            if dump_fn is not None:
                dump_fn.write(data)

except KeyboardInterrupt:
    print("\nDone")
except Exception as e:
    parser.exit(type(e).__name__ + ": " + str(e))
