#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction as FJTA
from control_msgs.msg import FollowJointTrajectoryGoal as FJTGoal
from trajectory_msgs.msg import JointTrajectoryPoint as JTPoint

def feedback_cb(feedback):
    pass

def fjt_action_client():
    client = actionlib.SimpleActionClient('FollowJointTrajectory', FJTA)

    client.wait_for_server()

    goal = FJTGoal()
    joints = ["joint1", "joint2"]

    point1 = JTPoint()
    point1.positions.extend([0.2, 0.5])

    point2 = JTPoint()
    point2.positions.extend([0.15, 3.14])

    point3 = JTPoint()
    point3.positions.extend([0.2, 0.5])

    point4 = JTPoint()
    point4.positions.extend([0.367, 6.2])

    point5 = JTPoint()
    point5.positions.extend([0.0, 0.0])
    
    goal.trajectory.points.extend([point1, point2, point3, point4, point5])
    goal.trajectory.joint_names.extend(joints)

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result


if __name__ == '__main__':
    try:
        rospy.init_node('follow_joint_trajectory_action_client')
        result = fjt_action_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")