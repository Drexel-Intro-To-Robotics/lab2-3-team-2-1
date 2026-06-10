#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import moveit_commander
import sys
import moveit_msgs.msg 


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
        self.group.set_goal_joint_tolerance(0.05)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)

        # Data recording
        self.measured_pos  = []
        self.measured_vel  = []
        self.measured_time = []

        # subscribe to joint states for logging
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_space_cb)

        rospy.loginfo("PolyTrajNode ready.")

    def joint_space_cb(self, msg):
        # Skip wheel_left_joint and wheel_right_joint (indices 0,1)
        self.measured_pos.append(list(msg.position[2:6]))
        self.measured_vel.append(list(msg.velocity[2:6]))
        self.measured_time.append(msg.header.stamp.to_sec())

    #clear recording
    def clear_log(self):
        self.measured_pos  = []
        self.measured_vel  = []
        self.measured_time = []

    # build and send goal
    def execute_trajectory(self, times, positions, velocities):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        
        # Force tolerance every time before executing
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.5)
        
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = rospy.Time.now()

        for i, t in enumerate(times):
            pt = JointTrajectoryPoint()
            pt.positions  = list(positions[i])
            pt.velocities = list(velocities[i])
            pt.time_from_start = rospy.Duration(float(t))
            traj.points.append(pt)

        moveit_traj = moveit_msgs.msg.RobotTrajectory()
        moveit_traj.joint_trajectory = traj

        self.clear_log()

        # Retry loop - resync start state each attempt
        max_attempts = 5
        for attempt in range(max_attempts):
            self.group.set_start_state_to_current_state()
            rospy.sleep(0.5)
            
            # Rebuild trajectory starting from actual current position
            current_q = list(self.group.get_current_joint_values())
            traj.points[0].positions = current_q  # snap first point to current
            traj.header.stamp = rospy.Time.now()
            moveit_traj.joint_trajectory = traj
            
            result = self.group.execute(moveit_traj, wait=True)
            self.group.stop()
            
            if result:
                rospy.loginfo(f"Execution succeeded on attempt {attempt+1}")
                break
            else:
                rospy.logwarn(f"Attempt {attempt+1} failed, retrying...")
                rospy.sleep(1.0)
        else:
            rospy.logerr("All execution attempts failed.")

        rospy.sleep(0.5)

    # Task 9 - compute polynomial in task space, convert to joint space via IK
    def run_task_space(self, x_start, x_end, T=4.0, method="quintic"):
        rospy.loginfo(f"Task 9: {method} task-space trajectory")
        # Read actual current position instead of assuming home
        q_start = list(self.group.get_current_joint_values())
        rospy.loginfo(f"Starting from actual joints: {np.round(q_start, 3)}")
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
        # Resync before execution
        self.group.set_start_state_to_current_state()
        rospy.sleep(1.0)
        self.execute_trajectory(times, q_arr, jvel)
        self.execute_trajectory(times, q_arr, jvel)

    # Task 10: polynomial trajectory
    def run_joint_space(self, q_start, q_end, T=4.0, method="quintic"):
        rospy.loginfo(f"Task 10: {method} joint-space trajectory")
        # Read actual current position instead of assuming home
        q_start = list(self.group.get_current_joint_values())
        rospy.loginfo(f"Starting from actual joints: {np.round(q_start, 3)}")
        # generate trajectory
        times, positions, velocities = generate_trajectory(q_start, q_end, T, method=method)

        #store and execute
        self.desired_pos  = positions
        self.desired_vel  = velocities
        self.desired_time = times
        self.group.set_start_state_to_current_state()
        rospy.sleep(1.0)
        self.execute_trajectory(times, positions, velocities)
        self.execute_trajectory(times, positions, velocities)

    def go_home(self):
        self.group.set_joint_value_target([0.017, -1.0, 0.297, 0.244])
        self.group.go(wait=True)
        self.group.stop()
        rospy.sleep(1.0)

if __name__ == "__main__":
    node = PolyTrajNode()
    rospy.sleep(1.0)

    q_home = [0.017, -1.0, 0.297, 0.244]
    q_goal = [0.017, 0.716, -0.803, -0.419]

    # Task 9: task-space quintic
    node.run_task_space(
        x_start = np.array([0.20, 0.0, 0.25]),
        x_end = np.array([0.25, 0.0, 0.35]),
        T=4.0, method="quintic")

    node.go_home()
    rospy.sleep(3.0)
    rospy.loginfo(f"==========Starting Task 10=================")
    # Task 10: joint-space quintic (same goal)
    node.run_joint_space(q_home, q_goal, T=4.0, method="quintic")
    node.go_home()
    rospy.loginfo(f"==========DONE=================")
