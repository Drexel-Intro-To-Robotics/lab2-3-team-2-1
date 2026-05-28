#!/usr/bin/env python3
import numpy as np

# DH Parameters [a, alpha, d, theta_offset] (HW3 Solution)
DH = np.array([
    [0.000, np.pi/2, 0.077,  0.0],
    [0.130, 0.0,     0.000,  0.0],
    [0.124, 0.0,     0.000,  0.0],
    [0.126, 0.0,     0.000,  0.0],
])

# DH transform matrix
def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ])

# forward kinematics
def forward_kinematics(q):
    T = np.eye(4)
    for i, (a, alpha, d, offset) in enumerate(DH):
        T = T @ dh_transform(a, alpha, d, q[i] + offset)
    return T

# Jacobian matrix
def jacobian(q, delta=1e-5):
    p0 = forward_kinematics(q)[:3, 3]
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.zeros(len(q))
        dq[i] = delta
        p1 = forward_kinematics(q + dq)[:3, 3]
        J[:, i] = (p1 - p0) / delta
    return J

def inv_kinematics(p_desired, q_init=None, alpha=0.8,
                        max_iter=2000, tol=1e-4, damping=1e-4):
    q = q_init.copy() if q_init is not None else np.zeros(4)

    for i in range(max_iter):
        p_curr = forward_kinematics(q)[:3, 3]
        err = p_desired - p_curr

        if np.linalg.norm(err) < tol:
            return q, True, i

        J = jacobian(q)
        # Damped least squares: J^T (J J^T + λI)^{-1}
        JJT = J @ J.T
        J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(3))
        dq = alpha * J_pinv @ err
        q = q + dq
        q = np.clip(q, -np.pi, np.pi)

    return q, False, max_iter

def smart_q_init(p_desired):
    x, y, z = p_desired
    q1 = np.arctan2(y, x)
    return np.array([q1, -np.pi/6, np.pi/3, -np.pi/6])

# Target positions [x, y, z] in meters
targets = {
    "Pose A (front, mid)":  np.array([0.25,  0.00,  0.15]),
    "Pose B (left side)":   np.array([0.15,  0.15,  0.20]),
    "Pose C (low, right)":  np.array([0.20, -0.10,  0.08]),
}

print(f"FK sanity check at q=0: {np.round(forward_kinematics(np.zeros(4))[:3,3], 4)}")
print("=" * 60)

results = {}  # store for use in arm commander later

for name, p_des in targets.items():
    best = None

    for attempt in range(20):
        if attempt == 0:
            q0 = smart_q_init(p_des)
        else:
            q0 = smart_q_init(p_des) + np.random.uniform(-0.5, 0.5, 4)
            q0 = np.clip(q0, -np.pi, np.pi)

        q_sol, success, iters = inv_kinematics(p_des, q_init=q0)
        p_check = forward_kinematics(q_sol)[:3, 3]
        err = np.linalg.norm(p_check - p_des)

        if best is None or err < best[2]:
            best = (q_sol, iters, err, success)
        if err < 1e-4:
            break

    q_sol, iters, pos_error, success = best
    p_check = forward_kinematics(q_sol)[:3, 3]
    results[name] = q_sol

    print(f"Target: {name}")
    print(f"Desired  position: {p_des}")
    print(f"Solved joints (rad): {np.round(q_sol, 4)}")
    print(f"Solved joints (deg): {np.round(np.degrees(q_sol), 2)}")
    print(f"FK-verified position: {np.round(p_check, 5)}")
    print(f"Position error: {pos_error:.6f} m")
    print(f"Converged: {success} in {iters} iterations")
    print("")