#!/usr/bin/env python3

"""
Skill that calls MonitorOrientation and rotate primitives to draw a square with the turtle :
"""

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


class ResetOrientation:
    """
    Class for DrawSquare skill.
    """

    # create messages that are used to publish feedback/result
    feedback_ = ACROBA_Workshop_SIGMA.msg.DrawSquareFeedback()
    result_ = ACROBA_Workshop_SIGMA.msg.DrawSquareResult()

    client_MonitorOrientation = actionlib.SimpleActionClient(
        "MonitorOrientation", ACROBA_Workshop_SIGMA.msg.MonitorOrientationAction
    )

    client_rotate = actionlib.SimpleActionClient(
        "Rotate", ACROBA_Workshop_SIGMA.msg.RotateAction
    )

    # create goal message
    goal_MonitorOrientation = ACROBA_Workshop_SIGMA.msg.MonitorOrientationGoal()
    goal_rotate = ACROBA_Workshop_SIGMA.msg.RotateGoal()

    def __init__(self, name):
        self.client_MonitorOrientation.wait_for_server()
        self.client_rotate.wait_for_server()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ACROBA_Workshop_SIGMA.msg.ResetOrientationAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server ready")

    def execute_cb(self, goal):
        """
        Callback function.
        """

        print("test")

        rate = rospy.Rate(60)
        success = True

        # define first goal
        self.goal_MonitorOrientation.turtle_name = goal.turtle_name

        # send goal
        self.client_MonitorOrientation.send_goal(self.goal_MonitorOrientation)

        # wait for response
        while self.client_MonitorOrientation.get_result() is None:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.client_MonitorOrientation.cancel_goal()
                self._as.set_preempted()
                success = False
                break
            rate.sleep()

        if self.client_MonitorOrientation.get_state() == 4:  # if goal aborted
            success = False
            rospy.loginfo("%s: Aborted" % self._action_name)
            self._as.set_aborted()

        # continu only if previous one success
        if success:
            result_MonitorOrientation = self.client_MonitorOrientation.get_result()
            # define next goal with previous result
            self.goal_rotate.degrees = result_MonitorOrientation.degrees
            self.goal_rotate.isClockwise = result_MonitorOrientation.isClockwise
            self.goal_rotate.speed = goal.speed

            # send goal
            self.client_rotate.send_goal(self.goal_rotate)

            # wait for response
            while self.client_rotate.get_result() is None:
                if self._as.is_preempt_requested() or rospy.is_shutdown():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self.client_rotate.cancel_goal()
                    self._as.set_preempted()
                    success = False
                    break
                rate.sleep()

            if self.client_rotate.get_state() == 4:  # if goal aborted
                success = False
                rospy.loginfo("%s: Aborted" % self._action_name)
                self._as.set_aborted()
        
        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result_)


if __name__ == "__main__":
    rospy.init_node("ResetOrientation")
    server = ResetOrientation(rospy.get_name())
    rospy.spin()
