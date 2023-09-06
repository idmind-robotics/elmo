#! /usr/bin/env python3


import akinator


import robot

import json
import rospy
from std_srvs.srv import Trigger


PROGRESSION_THRESHOLD = 80


class Node(object):
    def __init__(self):
        self.onboard = robot.Onboard()
        self.aki = None
        rospy.Service("/akinator/answer/yes", Trigger, self.on_yes)
        rospy.Service("/akinator/answer/no", Trigger, self.on_no)
        rospy.Service("/akinator/answer/probably", Trigger, self.on_probably)
        rospy.Service("/akinator/answer/probably_not", Trigger, self.on_probably_not)
        rospy.Service("/akinator/answer/maybe", Trigger, self.on_maybe)
        rospy.Service("/akinator/start", Trigger, self.on_start)

    def on_start(self, _):
        try:
            print("initializing akinator")
            self.aki = akinator.Akinator()
            print("starting game")
            self.aki.start_game()
            print("setting onboard")
            self.onboard.set_text(self.aki.question)
            print(self.aki.question)
            return True, self.aki.question
        except Exception as e:
            return False, e

    def __send_reply__(self, r):
        try:
            if not self.aki:
                return False, "game not running"
            q = self.aki.answer(r)
            if self.aki.progression > PROGRESSION_THRESHOLD:
                self.aki.win()
                # self.onboard.set_text(json.dumps(self.aki.first_guess))
                self.onboard.set_text("Is your character %s?" % self.aki.first_guess["name"])
                return True, json.dumps(self.aki.first_guess)
            else:
                self.onboard.set_text(q)
                return False, q
        except akinator.AkiConnectionFailure:
            return False, "AkiConnectionFailure"
        except akinator.AkiTimedOut:
            return False, "AkiTimedOut"
        except akinator.AkiNoQuestions:
            return False, "AkiNoQuestions"
        except akinator.AkiServerDown:
            return False, "AkiServerDown"
        except akinator.AkiTechnicalError:
            return False, "AkiTechnicalError"

    def on_yes(self, _):
        return self.__send_reply__('y')

    def on_no(self, _):
        return self.__send_reply__('n')

    def on_probably(self, _):
        return self.__send_reply__('p')

    def on_probably_not(self, _):
        return self.__send_reply__('pn')

    def on_maybe(self, _):
        return self.__send_reply__('i')


if __name__ == '__main__':
    rospy.init_node("akinator")
    NODE = Node()
    rospy.loginfo(rospy.get_name() + ": running")
    rospy.spin()
