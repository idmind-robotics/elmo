#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import spidev

spi = spidev.SpiDev()
spi.open(0,0)
spi.mode = 0b11
spi.max_speed_hz = 1000000
spi.bits_per_word = 8
spi.lsbfirst=False
start_frame = [0x00, 0x00, 0x00, 0x00]
end_frame = [0xFF, 0xFF, 0xFF, 0xFF]
LED_GREEN = [0xE1, 0x10, 0xD0, 0x35]
LED_OFF = [0xE1, 0x00, 0x00, 0x00]
var = 'non'

correctIconTemplate = [
    [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
    [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0]]

def buildIcon(template, color):
    icon=start_frame[:]
    for i in range(15,7,-1): #i=ligne
        for j in range(0,8): #j= colonne
            if template[i][j] == 0:
                icon += LED_OFF
            else:
                icon += color
        
    for i in range(7,-1,-1):
        for j in range(0,8):
            if template[i][j] == 0:
                icon += LED_OFF
            else:
                icon += color

    for i in range(15,7,-1):
        for j in range(8,16):
            if template[i][j] == 0:
                icon += LED_OFF
            else:
                icon += color

    for i in range(7,-1,-1):
        for j in range(8,16):
            if template[i][j] == 0:
                icon += LED_OFF
            else:
                icon += color

    icon += end_frame
    return icon

def callback(data):
    global var 
    if data.data == var:
        pass
    else :
        var = data.data
        affichage()

def affichage():
    correctIcon = buildIcon(correctIconTemplate, LED_GREEN)
    spi.writebytes(correctIcon)




def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("touch", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def cleanup():
    off = start_frame + [0xE0, 0x00, 0x00, 0x00] *256 + end_frame
    spi.writebytes(off)

if __name__ == '__main__':
    rospy.sleep(5.0)
    listener()
    rospy.on_shutdown(cleanup)