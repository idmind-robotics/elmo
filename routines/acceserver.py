#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
import os 


def run():
    path = '/home/elmo/catkin_ws/src/routines/scripts'
    os.chdir(path)
    os.system("python -m http.server 8000 ") #8081
    #os.system("firefox http://localhost:8000/ --kiosk")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass