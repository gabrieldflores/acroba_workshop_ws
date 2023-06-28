#!/usr/bin/env python3

"""
Skill that calls move and rotate primitives to Patrol with the turtle :
"""

import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg


class Patrol:
    """
    Class for Patrol skill.
    """

    # create messages that are used to publish feedback/result
    feedback_ = ACROBA_Workshop_SIGMA.msg.PatrolFeedback()
    result_ = ACROBA_Workshop_SIGMA.msg.PatrolResult()

    client_move = actionlib.SimpleActionClient(
        "Move", ACROBA_Workshop_SIGMA.msg.MoveAction
    )

    client_rotate = actionlib.SimpleActionClient(
        "Rotate", ACROBA_Workshop_SIGMA.msg.RotateAction
    )

    # create goal message
    goal_move = ACROBA_Workshop_SIGMA.msg.MoveGoal()
    goal_rotate = ACROBA_Workshop_SIGMA.msg.RotateGoal()

    def __init__(self, name):
        self.client_move.wait_for_server()
        self.client_rotate.wait_for_server()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ACROBA_Workshop_SIGMA.msg.PatrolAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server ready")

    def execute_cb(self, goal):
        """
        Callback function.
        """

        rate = rospy.Rate(60)
        success = True

        # define first goal
        self.goal_move.turtle_name = goal.turtle_name
        self.goal_move.speed = goal.speed_move
        self.goal_move.distance = goal.distance
        self.goal_move.isForward = goal.isForward

        # send goal
        self.client_move.send_goal(self.goal_move)

        # wait for response
        while self.client_move.get_result() is None:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.client_move.cancel_goal()
                self._as.set_preempted()
                success = False
                break
            rate.sleep()
        #rospy.sleep(0.5)

        if self.client_move.get_state() == 4:  # if goal aborted
            success = False
            rospy.loginfo("%s: Aborted" % self._action_name)
            self._as.set_aborted()

        # continu only if previous one success
        if success:
            # define next goal
            self.goal_rotate.turtle_name = goal.turtle_name
            self.goal_rotate.speed = goal.speed_rotate
            self.goal_rotate.degrees = goal.degrees
            self.goal_rotate.isClockwise = goal.isClockwise

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
            rospy.sleep(0.5)

            if self.client_rotate.get_state() == 4:  # if goal aborted
                success = False
                rospy.loginfo("%s: Aborted" % self._action_name)
                self._as.set_aborted()
        
        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result_)


if __name__ == "__main__":
    rospy.init_node("Patrol")
    server = Patrol(rospy.get_name())
    rospy.spin()
