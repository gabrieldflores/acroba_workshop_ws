#!/usr/bin/env python3

import rospy
#from __future__ import print_function
import actionlib
import actionlib_tutorials.msg

# define global variables if needed

def main():
    rospy.init_node("fibonacci_client", anonymous=True)
    # define publisher or subscriber if needed
   # pub = rospy.Publisher("topic", message, queue_size=1)
    #rospy.Subscriber("topice", message, callback)

    # define client
    client = actionlib.SimpleActionClient(
        "Fibonacci", actionlib_tutorials.msg.FibonacciAction
    )
    client.wait_for_server()

    goal = actionlib_tutorials.msg.FibonacciGoal()

    goal.order = 2
    goal.seed1 = 0
    goal.seed2 = 1

    for i in range(0,46):
        try:
    # send goal and wait
            client.send_goal(goal)

            if client.wait_for_result():
                print("Success")

            result = client.get_result()
            print("Result:", ', '.join([str(n) for n in result.sequence]))

            goal.seed1 = result.sequence[len(result.sequence)-2]
            goal.seed2 = result.sequence[len(result.sequence)-1]

        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)


if __name__ == "__main__":
    main()




