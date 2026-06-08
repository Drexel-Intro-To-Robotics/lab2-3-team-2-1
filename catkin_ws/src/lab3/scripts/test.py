#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('go_to_pose', anonymous=True)

arm = moveit_commander.MoveGroupCommander("arm")
arm.set_planning_time(5.0)

# Read XYZ from command line args
x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

pose = Pose()
pose.position.x = x
pose.position.y = y
pose.position.z = z
pose.orientation.w = 1.0  # neutral orientation

arm.set_pose_target(pose)
success = arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

print(f"Motion {'succeeded' if success else 'FAILED'}")
moveit_commander.roscpp_shutdown()