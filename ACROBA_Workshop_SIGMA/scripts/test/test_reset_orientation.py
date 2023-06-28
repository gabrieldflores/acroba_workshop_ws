#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    rospy.init_node("test_ResetOrientation")

    client = actionlib.SimpleActionClient(
        "ResetOrientation", ACROBA_Workshop_SIGMA.msg.ResetOrientationAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.ResetOrientationGoal()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        goal.turtle_name = "turtle1"
        goal.speed = 90 #deg/s
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
