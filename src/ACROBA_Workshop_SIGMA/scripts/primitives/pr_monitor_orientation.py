#!/usr/bin/env python3
import rospy
import actionlib
import math
import ACROBA_Workshop_SIGMA.msg
from turtlesim.msg import Pose


class MonitorOrientationAction:
    # create messages that are used to publish feedback/result
    feedback = ACROBA_Workshop_SIGMA.msg.MonitorOrientationFeedback()
    result = ACROBA_Workshop_SIGMA.msg.MonitorOrientationResult()

    def __init__(self, name):
        self.initial_orientation = None
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ACROBA_Workshop_SIGMA.msg.MonitorOrientationAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server Ready...")

    # Callback function implementing the pose value received
    def pose_callback(self, data):
        self.pose = data
        self.pose.theta = round(self.pose.theta, 4)
        if self.initial_orientation is None:
            self.initial_orientation = self.pose.theta

    def execute_cb(self, goal):
        
        if not goal.turtle_name:
            self._as.set_aborted()
            rospy.loginfo("Aborted: turtle name shouldn't be empty")

        else:
            success = True
            rospy.Subscriber("/"+goal.turtle_name+"/pose", Pose, self.pose_callback)
            # Checking if the movement is clockwise or counter-clockwise
            if self.initial_orientation > 0:
                self.result.isClockwise = True
                self.result.degrees = math.degrees(self.initial_orientation)
            else:
                self.result.isClockwise = False
                self.result.degrees = -math.degrees(self.initial_orientation)

            if success:
                rospy.loginfo("%s: Succeeded" % self._action_name)
                self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("MonitorOrientation")
    server = MonitorOrientationAction(rospy.get_name())
    rospy.spin()
