#! /usr/bin/env python3

import rospy
import actionlib

# other needed imports
import actionlib_tutorials.msg

class Fibonacci(object):
    # create messages that are used to publish feedback/result
    feedback_ = actionlib_tutorials.msg.FibonacciFeedback()
    result_ = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        """
        Callback function.
        """
        '''''
        if (
            # test if given goals are in the required format / not missing
            # Example
            goal.goal1 is int
            or not goal.goal2
        ):
            self._as.set_aborted()
            if goal.goal1 is int:
                rospy.loginfo("Aborted: goal 1 is given in a wrong format")
            if not goal.goal2:
                rospy.loginfo("Aborted: info missing : empty goal 2")
        else:
        '''
        r = rospy.Rate(1)
        success = True

        self.feedback_.sequence = []
        self.feedback_.sequence.append(goal.seed1)
        self.feedback_.sequence.append(goal.seed2)

        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self.feedback_.sequence[0], self.feedback_.sequence[1]))

        for i in range(1, goal.order):
        # this block is needed to handle cancelation during processing
            if self._as.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False

            self.feedback_.sequence.append(self.feedback_.sequence[i] + self.feedback_.sequence[i - 1])
            self._as.publish_feedback(self.feedback_)

            # This block is needed at the end to send success and result data
        if success:
            self.result_.sequence = self.feedback_.sequence
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result_)


if __name__ == "__main__":
    rospy.init_node("Fibonacci")
    server = Fibonacci(rospy.get_name())
    rospy.spin()
