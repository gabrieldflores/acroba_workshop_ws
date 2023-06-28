#!/usr/bin/env python3
import rospy
import actionlib
import ACROBA_Workshop_SIGMA.msg
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from math import pow, atan2, sqrt


class GotogoalAction:
    # create messages that are used to publish feedback/result
    feedback = ACROBA_Workshop_SIGMA.msg.GotogoalFeedback()
    result = ACROBA_Workshop_SIGMA.msg.GotogoalResult()

    def __init__(self, name):
        self.vel_msg = Twist()
        self.pose = Pose()
        self.velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ACROBA_Workshop_SIGMA.msg.GotogoalAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server Ready...")

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def execute_cb(self, goal):
        goal_pose = Pose()
        goal_pose.x = goal.x
        goal_pose.y = goal.y

        if (goal.x is None or goal.y is None or goal.tolerance is None):
            self._as.set_aborted()
            if goal.x is None:
                rospy.loginfo("Aborted: x coordinate value should be specified")
            if goal.y is None:
                rospy.loginfo("Aborted: y coordinate value should be specified")
            if goal.tolerance is None:
                rospy.loginfo("Aborted: tolerance value should be specified")
        else:
            success = True
            rospy.loginfo("Let's move your robot to ")
            rospy.loginfo(str(Pose))

            while self.euclidean_distance(goal_pose) >= goal.tolerance:
                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control

                # Linear velocity in the x-axis.
                self.vel_msg.linear.x = self.linear_vel(goal_pose)
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                self.vel_msg.angular.z = self.angular_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(self.vel_msg)

            # Stopping our robot after the movement is over.
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.velocity_publisher.publish(self.vel_msg)

        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("Gotogoal")
    server = GotogoalAction(rospy.get_name())
    rospy.spin()
