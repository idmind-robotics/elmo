#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time
import board
import busio
import adafruit_mpr121

i2c = busio.I2C(board.SCL, board.SDA)
mpr121 = adafruit_mpr121.MPR121(i2c)

def node():
    pub = rospy.Publisher('touch', String, queue_size=10)
    rospy.init_node('node', anonymous=True)
    rate = rospy.Rate(0.5) # 0.5hz
    while not rospy.is_shutdown():
        for i in range(12):
            pub.publish("start")
        # Call is_touched and pass it then number of the input.  If it's touched
        # it will return True, otherwise it will return False.
            if mpr121[i].value:
                print("Input {} touched!".format(i))
                touched = "True"
                pub.publish(touched)
                rate.sleep()
        time.sleep(0.25)


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass