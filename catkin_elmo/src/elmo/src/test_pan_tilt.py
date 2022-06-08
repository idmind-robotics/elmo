#! /usr/bin/env python3


import random
import rospy
import robot


MIN_SLEEP = 1.0
MAX_SLEEP = 3.0
STARTUP_DELAY = 3.0


def main():
    rospy.init_node("test_pan_tilt")
    pt = robot.PanTilt()
    rospy.sleep(0.5)
    rospy.loginfo("testing pan and tilt in %d, ctrl+c to finish", STARTUP_DELAY)
    pt.enable(True, True)
    pt.reset_angles()
    rospy.sleep(STARTUP_DELAY)
    rospy.loginfo("starting tests")
    while not rospy.is_shutdown():
        limits = pt.get_limits()
        pt.set_angles(
            pan=random.uniform(limits["min_pan_angle"], limits["max_pan_angle"]),
            tilt=random.uniform(limits["min_tilt_angle"], limits["max_tilt_angle"])
        )
        status = pt.get_status()
        rospy.loginfo("pan temperature: %s", status["pan_temperature"])
        rospy.loginfo("tilt temperature: %s", status["tilt_temperature"])
        rospy.sleep(random.uniform(MIN_SLEEP, MAX_SLEEP))
    pt.reset_angles()
    rospy.loginfo("finish")


if __name__ == '__main__':
    main()