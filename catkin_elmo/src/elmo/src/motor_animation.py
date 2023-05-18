#! /usr/bin/env python3


"""

This node listens for motor positions.

It exposes a service to "start_recording". When this service is called, it starts recording the motor positions.

It exposes a service to "stop_recording". When this service is called, it stops recording the motor positions.

After recording, it generates a smooth curve with the recorded motor positions.

It exposes a service to "save_animation". When this service is called, it saves the animation to a file with the given name.

It exposes a service to "play_animation". When this service is called, it plays the animation.

It publishes the current animation name to the topic "animation_name".

It exposes a service to "stop_animation". When this service is called, it stops the animation.

"""


import numpy as np
#import matplotlib.pyplot as plt
from scipy.signal import savgol_filter


import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from elmo.msg import PanTilt
from elmo.srv import StringTrigger


PLOT_POINTS = False
SEND_MOTOR_COMMANDS = True
SMOOTH_WINDOW_LENGTH = 6
RATE_CALIBRATION_TIME = 3.0


class Node:

    def __init__(self):
        self.recording = False
        self.n_status_messages = 0
        t = rospy.Time.now()
        self.rate = None
        rospy.Subscriber("pan_tilt/status", PanTilt, self.on_status)
        rospy.loginfo(rospy.get_name() + ": Waiting for status messages for rate calibration...")
        while (rospy.Time.now() - t).secs < RATE_CALIBRATION_TIME:
            rospy.sleep(0.1)
        r = self.n_status_messages / RATE_CALIBRATION_TIME
        rospy.loginfo(rospy.get_name() + ": rate is %fHz." % r)
        self.rate = rospy.Rate(r)
        rospy.Service("~start_recording", Trigger, self.on_start_recording)
        rospy.Service("~stop_recording", Trigger, self.on_stop_recording)
        rospy.Service("~save_animation", StringTrigger, self.on_save_animation)
        rospy.Service("~play_animation", StringTrigger, self.on_play_animation)
        rospy.Service("~stop_animation", Trigger, self.on_stop_animation)
        self.is_playing_pub = rospy.Publisher("~is_playing", Bool, queue_size=10)
        self.animation_name = ""
        self.animation = None
        self.animation_index = 0
        self.animation_playing = False
        self.command_publisher = rospy.Publisher("pan_tilt/command_raw", PanTilt, queue_size=10)

    def on_status(self, msg):
        """ Motor status callback."""
        if self.rate is None:
            self.n_status_messages += 1
        if self.recording:
            self.animation.append((msg.pan_angle, msg.tilt_angle))

    def on_start_recording(self, msg):
        """ Start recording callback."""
        self.recording = True
        self.animation = []
        return True, "Recording started"

    def on_stop_recording(self, msg):
        """ Stop recording callback."""
        self.recording = False
        return True, "Recording stopped"

    def on_save_animation(self, msg):
        """ This function saves the animation to a file with the given name. """
        animation_name = msg.data
        if self.animation is not None:
            self.animation_name = animation_name
            if PLOT_POINTS:
                self.plot_animation()
            pan_angles = [int(x[0]) for x in self.animation]
            tilt_angles = [int(x[1]) for x in self.animation]
            pan_angles, tilt_angles = self.trim_angles(pan_angles, tilt_angles)
            pan_angles = self.smooth_points(pan_angles)
            tilt_angles = self.smooth_points(tilt_angles)
            self.animation = zip(pan_angles, tilt_angles)
            if PLOT_POINTS:
                self.plot_animation()
            with open(self.animation_name + ".txt", "w") as f:
                f.write("pan_angle, tilt_angle\n")
                for pan_angle, tilt_angle in self.animation:
                    f.write(str(pan_angle) + "," + str(tilt_angle) + "\n")
            return True, "Animation saved"
        else:
            return False, "No animation to save"

    def trim_angles(self, pan_angles, tilt_angles):
        """ This function takes two lists of float values and removes the first and last part of the lists that are equal. """
        if len(pan_angles) < 2 or len(tilt_angles) < 2:
            return pan_angles, tilt_angles
        else:
            first_pan_value = pan_angles[0]
            last_pan_value = pan_angles[-1]
            first_tilt_value = tilt_angles[0]
            last_tilt_value = tilt_angles[-1]
            for i in range(len(pan_angles)):
                if pan_angles[i] != first_pan_value or tilt_angles[i] != first_tilt_value:
                    break
            for j in range(len(pan_angles)-1, -1, -1):
                if pan_angles[j] != last_pan_value or tilt_angles[j] != last_tilt_value:
                    break
            print("trimming angles from %d to %d out of %d" %  (i, j, len(pan_angles)))
            return pan_angles[i:j+1], tilt_angles[i:j+1]

    def smooth_points(self, points):
        """ This function takes a list of float values and generates a smooth curve with those points. """
        if len(points) < SMOOTH_WINDOW_LENGTH:
            return points
        else:
            return savgol_filter(points, SMOOTH_WINDOW_LENGTH, 3)

    def plot_animation(self):
        """
        This function creates two plots, one for the pan angles and one for the tilt angles of the animation.
        The plots are limited by the maximum and minimum pan and tilt angles.
        """
        pan_angles = [x[0] for x in self.animation]
        tilt_angles = [x[1] for x in self.animation]
        plt.subplot(2, 1, 1)
        plt.plot(pan_angles)
        plt.subplot(2, 1, 2)
        plt.plot(tilt_angles)
        plt.show()

    def on_play_animation(self, msg):
        """ This function plays the animation. """
        try:
            animation_name = msg.data
            #self.animation = np.load(self.animation_name + ".npy", allow_pickle=True)
            with open(animation_name + ".txt", "r") as f:
                lines = f.readlines()
                self.animation = []
                for line in lines[1:]:
                    pan_angle, tilt_angle = line.split(",")
                    self.animation.append((float(pan_angle), float(tilt_angle)))
            self.animation_name = animation_name
            if self.animation is not None:
                self.animation_index = 0
                self.animation_playing = True
                return True, "Animation playing"
            else:
                return False, "No animation to play"
        except IOError:
            return False, "Animation not found"
            
    def on_stop_animation(self, msg):
        """ This function stops the animation. """
        self.animation_playing = False
        return True, "Animation stopped"

    def run(self):
        """ The main loop. Plays the animation if it is playing. """
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.animation_playing and self.animation is not None:
                self.animation_index += 1
                if self.animation_index >= len(self.animation):
                    self.animation_index = 0
                    self.animation_playing = False
                    self.is_playing_pub.publish(False)
                else:
                    self.is_playing_pub.publish(True)
                    pan_angle, tilt_angle = self.animation[self.animation_index]
                    if SEND_MOTOR_COMMANDS:
                        command = PanTilt()
                        command.pan_angle = pan_angle
                        command.tilt_angle = tilt_angle
                        command.pan_torque = True
                        command.tilt_torque = True
                        command.playtime = 50
                        self.command_publisher.publish(command)


if __name__ == '__main__':
    rospy.init_node('motor_animation')
    node = Node()
    rospy.loginfo(rospy.get_name() + ": is running")
    node.run()
