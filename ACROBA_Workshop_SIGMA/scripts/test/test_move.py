#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    rospy.init_node("test_Move")

    client = actionlib.SimpleActionClient(
        "Move", ACROBA_Workshop_SIGMA.msg.MoveAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.MoveGoal()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        goal.turtle_name = "turtle2"
        goal.speed = 1
        goal.distance = 2
        goal.isForward = True
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
