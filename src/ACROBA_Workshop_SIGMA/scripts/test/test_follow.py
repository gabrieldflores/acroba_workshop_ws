#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    # Creating our node,publisher and subscriber
    rospy.init_node("test_Gotogoal")
    client = actionlib.SimpleActionClient(
        "Gotogoal", ACROBA_Workshop_SIGMA.msg.GotogoalAction
    )
    client.wait_for_server()

    goal = ACROBA_Workshop_SIGMA.msg.GotogoalGoal()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        #goal.turtle1_name = 'turtle1'
        #goal.turtle2_name = 'turtle2'
        goal.tolerance = 1
        client.send_goal(goal)
        client.wait_for_result()
        rate.sleep()


if __name__ == "__main__":
    main()
