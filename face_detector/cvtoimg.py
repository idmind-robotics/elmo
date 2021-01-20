#!/usr/bin/env python2.7

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

liste=[0]
lv = [50]
rospy.set_param('lim_h_min', 500)
rospy.set_param('lim_h_max', 800)
rospy.set_param('lim_v_min', 90)
rospy.set_param('lim_v_max', 110)
rospy.set_param('middle_h', 650)
rospy.set_param('middle_v', 100)

class image_converter:

    def __init__(self):
        cascPath = ("/home/elmo/catkin_ws/src/face_detector/scripts/haarcascade_frontalface_default.xml")
        self.faceCascade = cv2.CascadeClassifier(cascPath)
        self.id_servo = rospy.Publisher("id_servo", Int16, queue_size=10)
        self.position = rospy.Publisher("position", Int16, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
        self.lim_h_max = rospy.get_param("lim_h_max")
        self.lim_h_min = rospy.get_param("lim_h_min")
        self.lim_v_max = rospy.get_param("lim_v_max")
        self.lim_v_max = rospy.get_param("lim_v_min")
        self.middle_h = rospy.get_param("middle_h")
        self.middle_v = rospy.get_param("middle_v")


    def callback(self,data):
        global liste
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Detection visage
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(gray, 1.1, 4)
        print("faces:",faces)

        if len(faces) == 0:
            print("No faces detected")
        else:
            # Mouvement horizontal
            h=faces[0][0]
            ref = liste[-1]
            if h < ref+10 and h > ref-10:
                liste.append(h)
            else:
                liste.append(h)
                self.horizontal_adjustement(h)
                rospy.sleep(0.5)

            # Mouvement vertical
            v=faces[0][1]
            sim = lv[-1]
            if v < sim+5 and v > sim-5:
                lv.append(v)
            else:
                if v > 100 or v < 10:
                    self.vertical_adjustment(v)
                    rospy.sleep(0.5)
                else:
                    pass 


        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def horizontal_adjustement(self, msg):
        self.id_servo.publish(4)
        if msg < 300 and msg > 200 :
             print("centre")
        elif msg > 300:
            self.middle_h+=10 
            print("moyenne", self.middle_h)
            if self.middle_h < self.lim_h_max:
                self.position.publish(self.middle_h)
            else:
                print("of limits")
        elif msg < 200:
            self.middle_h-=10
            print("moyenne", self.middle_h)
            if self.middle_h > self.lim_h_min:
                self.position.publish(self.middle_h)
            else:
                print("of limits")

    def vertical_adjustment(self, msg):
        self.id_servo.publish(3)
        if msg > 100 and self.middle_v > self.lim_v_min:
            self.middle_vmiddle-=10
            print("position servo expected", self.middle_v)
            self.position.publish(self.middle_v)
        elif msg < 10 and self.middle_v < self.lim_v_max:
            self.middle_v+=10
            print("position servo expected", self.middle_v)
            self.position.publish(self.middle_v)
        else:
            print("out of limits")


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)