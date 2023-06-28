#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    rospy.init_node("test_Rotate")

    client = actionlib.SimpleActionClient(
        "Rotate", ACROBA_Workshop_SIGMA.msg.RotateAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.RotateGoal()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        goal.turtle_name = "turtle1"
        goal.speed = 90 #deg/s
        goal.degrees = 180 #deg
        goal.isClockwise = False
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
