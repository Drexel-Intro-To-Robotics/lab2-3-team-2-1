#!/usr/bin/env python3
import argparse
import math
import numpy as np


DH_PARAMS = [
    # a (m)    alpha (rad)   d (m)    theta_offset (rad)
    [0.0,      math.pi/2,    0.0770,  0.0          ],   # Joint 1
    [0.1284,   0.0,          0.0,    -math.pi/2    ],   # Joint 2
    [0.1240,   0.0,          0.0,     0.0          ],   # Joint 3
    [0.1260,   0.0,          0.0,     0.0          ],   # Joint 4 + tool
]

# Mounting transform: arm base relative to TurtleBot3 base_footprint
ARM_MOUNT_X = 0.0     # meters forward from base_footprint
ARM_MOUNT_Y = 0.0     # meters left from base_footprint
ARM_MOUNT_Z = 0.109   # meters above ground (Waffle Pi upper plate height)
ARM_MOUNT_ROLL  = 0.0
ARM_MOUNT_PITCH = 0.0
ARM_MOUNT_YAW   = 0.0


def dh_transform(a, alpha, d, theta):
    """
    Compute the 4x4 homogeneous transformation matrix using
    standard DH convention.

    T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)

    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0,    sa,     ca,     d   ],
        [0,    0,      0,      1   ]
    ])


def rpy_to_matrix(roll, pitch, yaw):
    """Roll-Pitch-Yaw to 3x3 rotation matrix (extrinsic XYZ)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ])


def mount_transform():
    """
    4x4 transform from TurtleBot3 base_footprint to arm base link.
    """
    R = rpy_to_matrix(ARM_MOUNT_ROLL, ARM_MOUNT_PITCH, ARM_MOUNT_YAW)
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = ARM_MOUNT_X
    T[1, 3] = ARM_MOUNT_Y
    T[2, 3] = ARM_MOUNT_Z
    return T


def robot_base_transform(x, y, yaw):
    """
    4x4 transform from world frame to TurtleBot3 base_footprint.
    (flat ground assumed, so roll=pitch=0)
    """
    R = rpy_to_matrix(0, 0, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = 0.0
    return T


def forward_kinematics(joint_angles_rad):
    assert len(joint_angles_rad) == 4, "Need exactly 4 joint angles."

    T = np.eye(4)
    T_mats = []

    for i, (a, alpha, d, theta_offset) in enumerate(DH_PARAMS):
        theta = joint_angles_rad[i] + theta_offset
        Ti = dh_transform(a, alpha, d, theta)
        T = T @ Ti
        T_mats.append(T.copy())

    xyz = T[:3, 3]

    # Extract RPY from rotation matrix
    R = T[:3, :3]
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(math.cos(pitch)) > 1e-6:
        roll  = math.atan2(R[2, 1] / math.cos(pitch), R[2, 2] / math.cos(pitch))
        yaw   = math.atan2(R[1, 0] / math.cos(pitch), R[0, 0] / math.cos(pitch))
    else:
        roll = 0.0
        yaw  = math.atan2(-R[0, 1], R[1, 1])

    return {
        "T_arm":   T,
        "xyz_arm": tuple(xyz),
        "rpy_arm": (roll, pitch, yaw),
        "T_mats":  T_mats,
    }


def full_state_space(joint_angles_rad, robot_pose=None):
    """
    Compute end-effector position in all relevant frames.

    Args:
        joint_angles_rad: [j1, j2, j3, j4] in radians
        robot_pose:       (x, y, yaw) of TurtleBot3 in world frame,
                          or None to skip world-frame computation

    Returns:
        dict with xyz in arm frame, robot frame, and (optionally) world frame
    """
    fk = forward_kinematics(joint_angles_rad)

    T_mount = mount_transform()
    T_ee_in_arm = fk["T_arm"]

    # End-effector in robot base_footprint frame
    T_ee_in_robot = T_mount @ T_ee_in_arm
    xyz_robot = tuple(T_ee_in_robot[:3, 3])

    result = {
        "arm_frame":   fk["xyz_arm"],
        "robot_frame": xyz_robot,
        "rpy_arm":     fk["rpy_arm"],
        "T_arm":       T_ee_in_arm,
        "T_robot":     T_ee_in_robot,
    }

    if robot_pose is not None:
        rx, ry, ryaw = robot_pose
        T_robot_in_world = robot_base_transform(rx, ry, ryaw)
        T_ee_in_world = T_robot_in_world @ T_ee_in_robot
        result["world_frame"] = tuple(T_ee_in_world[:3, 3])
        result["T_world"] = T_ee_in_world

    return result


def print_results(joints, robot_pose=None):
    """Pretty-print the state space results."""
    result = full_state_space(joints, robot_pose)

    print("\n" + "="*55)
    print("  OpenMANIPULATOR-X Forward Kinematics")
    print("="*55)

    print("\n  Joint Angles:")
    for i, a in enumerate(joints):
        print(f"    Joint {i+1}: {math.degrees(a):8.3f} deg  ({a:.4f} rad)")

    print("\n  End-Effector Position — ARM BASE FRAME:")
    x, y, z = result["arm_frame"]
    print(f"    X = {x*1000:8.2f} mm  ({x:.5f} m)")
    print(f"    Y = {y*1000:8.2f} mm  ({y:.5f} m)")
    print(f"    Z = {z*1000:8.2f} mm  ({z:.5f} m)")

    r, p, yw = result["rpy_arm"]
    print(f"\n  End-Effector Orientation (RPY) — ARM BASE FRAME:")
    print(f"    Roll  = {math.degrees(r):8.3f} deg")
    print(f"    Pitch = {math.degrees(p):8.3f} deg")
    print(f"    Yaw   = {math.degrees(yw):8.3f} deg")

    print("\n  End-Effector Position — ROBOT BASE_FOOTPRINT FRAME:")
    x, y, z = result["robot_frame"]
    print(f"    X = {x*1000:8.2f} mm  ({x:.5f} m)")
    print(f"    Y = {y*1000:8.2f} mm  ({y:.5f} m)")
    print(f"    Z = {z*1000:8.2f} mm  ({z:.5f} m)")

    if "world_frame" in result:
        print("\n  End-Effector Position — WORLD FRAME:")
        x, y, z = result["world_frame"]
        rx, ry, ryaw = robot_pose
        print(f"    (Robot at x={rx}, y={ry}, yaw={math.degrees(ryaw):.1f}°)")
        print(f"    X = {x*1000:8.2f} mm  ({x:.5f} m)")
        print(f"    Y = {y*1000:8.2f} mm  ({y:.5f} m)")
        print(f"    Z = {z*1000:8.2f} mm  ({z:.5f} m)")

    print("\n" + "="*55 + "\n")

    return result


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Forward kinematics for TurtleBot3 + OpenMANIPULATOR-X",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Examples:
  # Home position (all zeros)
  python3 openmanipulator_fk.py

  # Custom joint angles in radians
  python3 openmanipulator_fk.py --joints 0.5 0.3 -0.5 0.2

  # With robot position in world frame
  python3 openmanipulator_fk.py --joints 0.0 0.5 -0.3 0.8 --base-pose 1.0 0.5 0.785

  # Use degrees instead of radians
  python3 openmanipulator_fk.py --joints 30 45 -30 15 --degrees
        """
    )
    parser.add_argument(
        "--joints", "-j",
        nargs=4, type=float,
        default=[0.0, 0.0, 0.0, 0.0],
        metavar=("J1", "J2", "J3", "J4"),
        help="Four joint angles (default: 0 0 0 0)"
    )
    parser.add_argument(
        "--degrees", "-d",
        action="store_true",
        help="Interpret joint angles as degrees (default: radians)"
    )
    parser.add_argument(
        "--base-pose", "-p",
        nargs=3, type=float,
        metavar=("X", "Y", "YAW"),
        help="Robot base pose in world frame: x(m) y(m) yaw(rad)\n"
             "Enables world-frame output."
    )

    args = parser.parse_args()

    joints = args.joints
    if args.degrees:
        joints = [math.radians(a) for a in joints]

    robot_pose = None
    if args.base_pose:
        robot_pose = tuple(args.base_pose)

    print_results(joints, robot_pose)


if __name__ == "__main__":
    main()