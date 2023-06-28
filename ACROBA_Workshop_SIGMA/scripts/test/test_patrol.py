#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    rospy.init_node("test_patrol", anonymous=True)

    client = actionlib.SimpleActionClient(
        "Patrol", ACROBA_Workshop_SIGMA.msg.PatrolAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.PatrolGoal()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        goal.turtle_name = "turtle2"
        goal.speed_move = 100
        goal.distance = 2
        goal.isForward = True
        goal.speed_rotate = 200
        goal.degrees = 30
        goal.isClockwise = True
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
