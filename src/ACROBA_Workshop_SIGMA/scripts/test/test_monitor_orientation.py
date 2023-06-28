#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


def main():
    rospy.init_node("test_MonitorOrientation")

    client = actionlib.SimpleActionClient(
        "MonitorOrientation", ACROBA_Workshop_SIGMA.msg.MonitorOrientationAction
    )
    client.wait_for_server()
    goal = ACROBA_Workshop_SIGMA.msg.MonitorOrientationGoal()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        goal.turtle_name = "turtle1"
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print(result.degrees)
        print(result.isClockwise)
        rate.sleep()


if __name__ == "__main__":
    main()
