#! /usr/bin/env python2.7

import rospy
from std_msgs.msg import String
import time


class Node(object):
    
    def __init__(self):
        herkulex.connect("/dev/ttyAMA0", 115200)
        herkulex.clear_errors()
        rospy.Subscriber("routine", String, self.do_routine)
        self.sound = rospy.Publisher("sound", String, queue_size=10)
        self.eyes = rospy.Publisher("background", String, queue_size=10)
        self.id_servo = rospy.Publisher("id_servo", Int16, queue_size=10)
        self.position = rospy.Publisher("position", Int16, queue_size=10)
        self.led = rospy.Publisher("led", String, queue_size=10)

    def process_data(self, f):
        with open(f, "r") as file:
            dico = {}
            l_conso = []
            i = 0

            data = file.read()
            d = data.split()  # Splits into a list

            keys = d[::2]
            values = d[1::2]

            for i in range(len(keys)):
                l_i = ()
                l_i = keys[i], values[i]
                l_conso.append(l_i)

            for key, value in l_conso:
                if key not in dico:
                    dico[key] = [value]
                else:
                    dico[key].append(value)
            print(dico)
            return(dico) 


    def do_routine(self, data):
        routine = self.process_data("/home/elmo/catkin_ws/src/routines/scripts/yes.routine")
        delay = routine.get('delay:')
        i = 0
        for i in range(len(delay)) :
            time.sleep(float(delay[i]))
            cmd = routine.get('head_lr:')
            if cmd != None:
                self.id_servo.publish(4)
                self.position.publish(cmd[i])
            head = routine.get('head_ud:')
            if head != None:
                self.id_servo.publish(3)
                self.position.publish(head[i])
            icon = routine.get('icon:')
            if icon != None:
                self.led.publish(icon[i])
            sd = routine.get('sound:')
            if sd != None  and sd[i] != "none":
                print(sd[i])
                self.sound.publish(sd[i])
            look = routine.get('eyes:')
            if look != None:
                print(look[i])
                self.eyes.publish(look[i]) 

    

if __name__ == '__main__':
    rospy.init_node("routine")
    node = Node()
    rospy.loginfo("idmind_routine: running")
    rospy.spin()