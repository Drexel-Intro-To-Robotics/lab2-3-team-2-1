#!/usr/bin/env python3
"""
go_to_poses.py  –  TurtleBot3 Manipulation, ROS Noetic
Record with:  rosbag record -a -O my_bag.bag
"""

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

ARM_GROUP = "arm"

POSES = [
    {"name": "Pose 1", "x": 0.200, "y":  0.000, "z": 0.200, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    {"name": "Pose 2", "x": 0.180, "y":  0.050, "z": 0.180, "roll": 0.0, "pitch": 0.5, "yaw": 0.0},
    {"name": "Pose 3", "x": 0.220, "y":  0.000, "z": 0.150, "roll": 0.0, "pitch": 0.8, "yaw": 0.0},
]


def make_pose(p) -> Pose:
    q = quaternion_from_euler(p["roll"], p["pitch"], p["yaw"])
    pose = Pose()
    pose.position.x    = p["x"];  pose.position.y    = p["y"];  pose.position.z    = p["z"]
    pose.orientation.x = q[0];    pose.orientation.y = q[1]
    pose.orientation.z = q[2];    pose.orientation.w = q[3]
    return pose


def go_home(arm):
    rospy.loginfo("-> HOME")
    arm.set_named_target("home")
    arm.go(wait=True)
    arm.stop()
    rospy.sleep(5.0)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("go_to_poses", anonymous=True)

    arm = moveit_commander.MoveGroupCommander(ARM_GROUP)
    arm.set_max_velocity_scaling_factor(0.2)
    arm.set_max_acceleration_scaling_factor(0.2)
    arm.set_planning_time(10.0)
    arm.set_num_planning_attempts(10)

    rospy.loginfo("Starting in 3 s – make sure rosbag is running!")
    rospy.sleep(3.0)

    go_home(arm)

    for p in POSES:
        rospy.loginfo(f"=== {p['name']} ===")
        arm.set_pose_target(make_pose(p))
        success = arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

        if success:
            rospy.loginfo(f"  Reached – holding 2 s")
            rospy.sleep(2.0)
        else:
            rospy.logwarn(f"  FAILED – skipping")

        go_home(arm)

    rospy.loginfo("=== All done ===")
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()