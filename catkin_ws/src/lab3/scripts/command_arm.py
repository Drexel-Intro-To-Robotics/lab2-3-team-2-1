#!/usr/bin/env python3

# task 2 tester

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Paste your solved values directly from IK output above
POSES = {
    "Pose A (front, mid)":  [-0.0000, -0.8449,  1.9876, -0.6697],
    "Pose B (left side)":   [ 0.7854, -0.7172,  2.1055, -0.6316],
    "Pose C (low, right)":  [-0.4636, -1.3126,  2.3155, -0.8098],
}

HOME = [0.0, 0.0, 0.0, 0.0]

def move_to(client, joint_angles, duration=3.0, label=""):
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = joint_angles
    pt.velocities = [0.0] * 4
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    goal.trajectory = traj

    rospy.loginfo(f"Moving to: {label}")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(duration + 2.0))
    rospy.loginfo(f"Done: {label}")

if __name__ == '__main__':
    rospy.init_node('ik_commander')

    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    rospy.loginfo("Waiting for arm controller...")
    client.wait_for_server()
    rospy.loginfo("Connected to arm controller!")

    # Start from home
    move_to(client, HOME, duration=2.0, label="HOME")
    rospy.sleep(1.5)

    for name, q in POSES.items():
        move_to(client, q, duration=3.0, label=name)
        rospy.sleep(2.0)  # pause so you can observe/record each pose

        # Return to home between poses (required by lab spec Task 4a)
        move_to(client, HOME, duration=2.0, label="HOME")
        rospy.sleep(1.5)

    rospy.loginfo("All poses complete.")