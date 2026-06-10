#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# need to start up roscore, ssh into robot, and launch file
# rosrun lab3 task8_traj.py

# open manipulator joints
joint_names = ["joint1", "joint2", "joint3", "joint4"]

class TrajectoryNode:
    def __init__(self):
        #launch moveit and initialize node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_node", anonymous=True)

        # limit speed for testing - 40%
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)
        # set joint tolerance for reaching goals
        self.group.set_goal_joint_tolerance(0.01)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.05)

        # Publisher for trajectory visualization
        self.traj_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)

    # Joint space goal - q= [joint1, joint2, joint3, joint4]
    def move_to_joint_goal(self, q):
        rospy.loginfo(f"Joint-space goal: {np.round(q, 3)}")
        self.group.set_joint_value_target(q)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not success:
            rospy.logwarn("motion failed.")
        return success

    def move_to_task_goal(self, x, y, z, qx=0, qy=0, qz=0, qw=1):
        rospy.loginfo(f"Task-space goal: ({x:.3f}, {y:.3f}, {z:.3f})")
        target = geometry_msgs.msg.Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.x = qx
        target.orientation.y = qy
        target.orientation.z = qz
        target.orientation.w = qw
        self.group.set_pose_target(target)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not success:
            rospy.logwarn("motion failed.")
        return success

    # waypoint list
    def move_through_waypoints(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        rospy.loginfo(f"Executing {len(waypoints)} waypoints...")
        
        # Resync start state to current robot state
        self.group.set_start_state_to_current_state()
        rospy.sleep(0.5)  # let state update
        
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, eef_step, jump_threshold)
        
        rospy.loginfo(f"Cartesian path coverage: {fraction*100:.1f}%")
        if fraction > 0.9:
            # Retime the trajectory to smooth it out
            robot = moveit_commander.RobotCommander()
            retimed = self.group.retime_trajectory(
                robot.get_current_state(),
                plan,
                velocity_scaling_factor=0.3)
            self.group.execute(retimed, wait=True)
            self.group.stop()
        else:
            rospy.logwarn(f"Only {fraction*100:.1f}% of path achievable.")

    # Publish jointrajectory message (no moveit)
    def publish_joint_trajectory(self, q_list, time_list):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = rospy.Time.now()

        for q, t in zip(q_list, time_list):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            pt.velocities = [0.0] * 4
            pt.time_from_start = rospy.Duration(t)
            traj.points.append(pt)

        self.traj_pub.publish(traj)
        rospy.loginfo(f"Published trajectory with {len(q_list)} points.")


if __name__ == "__main__":
    node = TrajectoryNode()
    rospy.sleep(1.0)

    # Joint-space goal
    node.move_to_joint_goal([0.3, -0.5, 0.8, -0.3])
    rospy.sleep(1.0)

    # Task-space goal
    ry = np.pi / 6
    node.move_to_task_goal(-0.25, -0.4, -0.2, qy=np.sin(ry/2), qw=np.cos(ry/2))
    rospy.sleep(1.0)

    # Waypoint list (straight line forward)
    wps = []
    for i in range(5):
        p = Pose()
        p.position.x = 0.20 + i * 0.01
        p.position.y = 0.0
        p.position.z = 0.15
        p.orientation.w = 1.0
        wps.append(p)
    node.move_through_waypoints(wps)