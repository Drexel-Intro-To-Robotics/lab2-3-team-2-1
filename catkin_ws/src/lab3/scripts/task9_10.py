#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import moveit_commander
import sys


# setup - roscore, ssh, launch file (should see "you can start planning now")
# rosrun lab3 task9_10.py


# task 9 - compute the polynomial in cartesian space then convert to joints via IK
# task 10 - compute polynomial directly in joint space

#names on openmanipulator
joint_names = ["joint1", "joint2", "joint3", "joint4"]


# polynomial math functions for trajectory generation

# solve for cubic polynomial coefficients from BCs
def cubic_coeffs(q0, qf, v0, vf, T):
    a0 = q0
    a1 = v0
    a2 = (3*(qf - q0) - T*(2*v0 + vf)) / T**2
    a3 = (2*(q0 - qf) + T*(v0 + vf))   / T**3
    return np.array([a0, a1, a2, a3])

# Solve for quintic polynomial coefficients from BCs
def quintic_coeffs(q0, qf, v0, vf, a0_val, af, T):
    c0 = q0
    c1 = v0
    c2 = a0_val / 2.0
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    A = np.array([
        [T3,    T4,    T5   ],
        [3*T2,  4*T3,  5*T4 ],
        [6*T,   12*T2, 20*T3],
    ])
    b = np.array([
        qf - c0 - c1*T - c2*T2,
        vf - c1 - 2*c2*T,
        af - 2*c2,])
    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5])

# evaluate polynomial position at time t
def eval_poly(coeffs, t):
    return sum(c * t**i for i, c in enumerate(coeffs))

# evaluate polynomial velocity at time t
def eval_poly_vel(coeffs, t):
    return sum(i * c * t**(i-1) for i, c in enumerate(coeffs) if i > 0)

#generate trajectory
def generate_trajectory(q_start, q_end, T, dt=0.02, method="quintic"):
    times = np.arange(0, T + dt, dt)
    n_dof = len(q_start)
    positions  = np.zeros((len(times), n_dof))
    velocities = np.zeros((len(times), n_dof))
    
    # loop through joints
    for j in range(n_dof):
        if method == "cubic":
            c = cubic_coeffs(q_start[j], q_end[j], 0.0, 0.0, T)
        else:  # quintic
            c = quintic_coeffs(q_start[j], q_end[j], 0.0, 0.0, 0.0, 0.0, T)

        for i, t in enumerate(times):
            positions[i, j]  = eval_poly(c, t)
            velocities[i, j] = eval_poly_vel(c, t)
    return times, positions, velocities


# --------ROS Node------------

class PolyTrajNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("poly_traj_node", anonymous=True)

        #moveit! setup
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        self.client.wait_for_server()

        # Data recording
        self.measured_pos  = []
        self.measured_vel  = []
        self.measured_time = []

        # subscribe to joint states for logging
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_space_cb)

        rospy.loginfo("PolyTrajNode ready.")

    def joint_space_cb(self, msg):
        self.measured_pos.append(list(msg.position[:4]))
        self.measured_vel.append(list(msg.velocity[:4]))
        self.measured_time.append(msg.header.stamp.to_sec())

    #clear recording
    def clear_log(self):
        self.measured_pos  = []
        self.measured_vel  = []
        self.measured_time = []

    # build and send goal
    def execute_trajectory(self, times, positions, velocities):
        # setup FJTG message
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

        # send positions and velocities to ros_control
        for i, t in enumerate(times):
            pt = JointTrajectoryPoint()
            pt.positions = list(positions[i])
            pt.velocities = list(velocities[i])
            pt.time_from_start = rospy.Duration(float(t))
            goal.trajectory.points.append(pt)

        self.clear_log()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.sleep(0.5)

    # Task 9 - compute polynomial in task space, convert to joint space via IK
    def run_task_space(self, x_start, x_end, T=4.0, method="quintic"):
        rospy.loginfo(f"Task 9: {method} task-space trajectory")
        # cartesian trajectory
        times, positions, velocities = generate_trajectory(x_start, x_end, T, method=method)

        # Convert each waypoint to joint space via moveit! IK
        q_traj = []
        q_prev = list(self.group.get_current_joint_values())
        # convert cartesian point to joint angles
        for pos in positions:
            self.group.set_position_target(pos.tolist())
            plan = self.group.plan()
            if plan[0] and len(plan[1].joint_trajectory.points) > 0:
                q = list(plan[1].joint_trajectory.points[-1].positions)
            else:
                q = q_prev  # last known good config
            q_traj.append(q)
            q_prev = q

        #smooth out joint velocities
        q_arr = np.array(q_traj)
        # Recompute smooth velocities in joint space
        _, _, jvel = generate_trajectory(q_arr[0], q_arr[-1], T, method=method)

        #store and execute
        self.desired_pos = q_arr
        self.desired_vel = jvel
        self.desired_time = times
        self.execute_trajectory(times, q_arr, jvel)

    # Task 10: polynomial trajectory
    def run_joint_space(self, q_start, q_end, T=4.0, method="quintic"):
        rospy.loginfo(f"Task 10: {method} joint-space trajectory")
        # generate trajectory
        times, positions, velocities = generate_trajectory(q_start, q_end, T, method=method)

        #store and execute
        self.desired_pos  = positions
        self.desired_vel  = velocities
        self.desired_time = times
        self.execute_trajectory(times, positions, velocities)

    # Plotting
    def plot_results(self, title="Trajectory", save_path="/tmp/"):
        # sanity check
        if not self.measured_time:
            rospy.logwarn("No data recorded.")
            return

        # initalize variables for plotting
        t_meas = np.array(self.measured_time)
        t_meas -= t_meas[0]  # normalize to start at 0
        p_meas = np.array(self.measured_pos)
        v_meas = np.array(self.measured_vel)
        t_des = self.desired_time
        p_des = self.desired_pos
        v_des = self.desired_vel

        # Interpolate desired onto measured timestamps for error
        p_des_interp = np.array([
            np.interp(t_meas, t_des, p_des[:, j])
            for j in range(4)]).T
        v_des_interp = np.array([
            np.interp(t_meas, t_des, v_des[:, j])
            for j in range(4)]).T

        tracking_err = p_meas - p_des_interp

        joint_labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4"]

        # Figure 1 - position
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"{title} — Joint Position", fontweight="bold")
        for j, ax in enumerate(axes.flat):
            ax.plot(t_des,  p_des[:, j],  "b--", label="Desired",  linewidth=2)
            ax.plot(t_meas, p_meas[:, j], "r-",  label="Measured", linewidth=1.5)
            ax.set_title(joint_labels[j])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (rad)")
            ax.legend()
            ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f"{save_path}{title.replace(' ','_')}_position.png", dpi=150)
        plt.show()

        # Figure 2 - velocity
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"{title} — Joint Velocity", fontweight="bold")
        for j, ax in enumerate(axes.flat):
            ax.plot(t_des,  v_des[:, j],  "b--", label="Desired",  linewidth=2)
            ax.plot(t_meas, v_meas[:, j], "r-",  label="Measured", linewidth=1.5)
            ax.set_title(joint_labels[j])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Velocity (rad/s)")
            ax.legend()
            ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f"{save_path}{title.replace(' ','_')}_velocity.png", dpi=150)
        plt.show()

        # Figure 3 - Tracking error
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"{title} — Tracking Error", fontweight="bold")
        for j, ax in enumerate(axes.flat):
            ax.plot(t_meas, tracking_err[:, j], "g-", linewidth=1.5)
            ax.axhline(0, color="k", linestyle="--", linewidth=0.8)
            ax.fill_between(t_meas, tracking_err[:, j], alpha=0.2, color="g")
            ax.set_title(joint_labels[j])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Error (rad)")
            ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f"{save_path}{title.replace(' ','_')}_error.png", dpi=150)
        plt.show()

        # RMS error summary
        rms = np.sqrt(np.mean(tracking_err**2, axis=0))
        print(f"\n{'='*40}")
        print(f"RMS Tracking Error — {title}")
        for j, label in enumerate(joint_labels):
            print(f"  {label}: {rms[j]:.5f} rad")
        print(f"{'='*40}\n")

if __name__ == "__main__":
    node = PolyTrajNode()
    rospy.sleep(1.0)

    q_home = [0.0, 0.0, 0.0, 0.0]
    q_goal = [0.3, -0.5, 0.8, -0.3]

    # Task 9: task-space quintic
    node.run_task_space(
        x_start=np.array([0.15, 0.0, 0.20]),
        x_end  =np.array([0.25, 0.0, 0.15]),
        T=4.0, method="quintic")
    node.plot_results(title="Task 9 Task-Space Quintic", save_path="/tmp/")

    rospy.sleep(1.0)

    # Task 10: joint-space quintic (same goal)
    node.run_joint_space(q_home, q_goal, T=4.0, method="quintic")
    node.plot_results(title="Task 10 Joint-Space Quintic", save_path="/tmp/")