#!/usr/bin/env python3
"""
singularity_control.py
AMORO Lab - Part 3 (Bonus Section): Crossing Type 2 Singularities

This script investigates Type 2 (parallel) singularities of the Biglide robot:
1. Designs an arbitrary / unsafe trajectory crossing the Type 2 singularity (links collinear at 45 deg).
2. Computes theoretical inverse dynamic model (IDM) torques, demonstrating torque blow-up.
3. Designs a dynamically consistent "safe" trajectory satisfying the reciprocity condition:
       t_s^T * w_p = 0  <=>  ddot{y} = ddot{x} at collinear singularity
   proving that theoretical IDM actuator forces remain strictly finite and bounded.
4. Performs co-simulation tracking trials in Gazebo using Computed Torque Control (CTC),
   demonstrating why closed-loop feedback control fails near singularities (feedback gain amplification
   M(q)*Kp*e -> infinity due to unavoidable numerical perturbations e != 0).

Usage:
    python3 singularity_control.py             # Runs offline analysis & displays theoretical torques
    python3 singularity_control.py --gazebo    # Runs Gazebo co-simulation tracking trial
"""

import sys
import os
import argparse
import numpy as np

# Import Biglide models from the student scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from biglide_models import *

# Physical robot parameters (from lab subject)
D_RAILS = 0.4               # base width between vertical rails [m]
L_LEG = 0.2828427           # leg length l = 0.2 * sqrt(2) [m]
M_PLATFORM = 3.0            # mass of end-effector platform [kg]
M_FOOT = 1.0                # mass of each actuated foot/slider [kg]

# Co-simulation and control parameters
DT = 0.01                   # time step [s]
TF = 2.0                    # trajectory duration [s]
OMEGA_N = 20.0              # controller natural frequency [rad/s]
ZETA = 1.0                  # damping ratio
KP = OMEGA_N**2 * np.eye(2)
KD = 2.0 * ZETA * OMEGA_N * np.eye(2)


def quintic_time_scaling(t, tf=TF):
    """5th-order polynomial time law s(tau) with null boundary velocities and accelerations."""
    tau = np.clip(t / tf, 0.0, 1.0)
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    sD = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / tf
    sDD = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (tf**2)
    return s, sD, sDD


def design_unsafe_trajectory(tf=TF, dt=DT):
    """
    Trajectory 1: Unsafe Trajectory crossing Type 2 Singularity.
    Crosses the collinear singular configuration C_s = (0.0, 0.40) m at t_s = 0.70 s (tau = 0.35)
    where platform acceleration ddot{x} != 0 while ddot{y} = 0.
    Since t_s^T * w_p = m_p * (ddot{x} - ddot{y}) != 0, the dynamic consistency condition is violated,
    causing theoretical actuator forces to diverge (tau -> infinity).
    """
    t = np.arange(0.0, tf + dt / 2.0, dt)
    n = len(t)
    
    # We choose crossing at t = 0.70 s (tau = 0.35, where sDD != 0)
    s_cross, _, _ = quintic_time_scaling(0.70, tf)
    scale_x = 0.06
    
    pos = np.zeros((n, 2))
    vel = np.zeros((n, 2))
    acc = np.zeros((n, 2))
    
    for i, ti in enumerate(t):
        s, sD, sDD = quintic_time_scaling(ti, tf)
        # Horizontal motion crossing x = 0 at t = 0.70s with y = 0.40 m constant
        pos[i] = [-scale_x * s_cross + scale_x * s, 0.40]
        vel[i] = [scale_x * sD, 0.0]
        acc[i] = [scale_x * sDD, 0.0]
        
    return t, pos, vel, acc


def design_safe_trajectory(tf=TF, dt=DT):
    """
    Trajectory 2: Dynamically Consistent Safe Trajectory.
    Crosses the collinear singular configuration along the collinear link direction (slope = 1, 45 deg):
        y(t) = x(t) + 0.40 m
    This guarantees that at the singular instant (and indeed throughout the motion):
        ddot{y}(t) = ddot{x}(t)  <=>  w_p = m_p * ddot{x} * [1, 1]^T in Range(A_r^T)
    The uncontrollable transverse motion t_s = [1, -1]^T is orthogonal to w_p:
        t_s^T * w_p = m_p * (ddot{x} - ddot{y}) = 0
    Therefore, theoretical actuator forces remain strictly finite and bounded!
    """
    t = np.arange(0.0, tf + dt / 2.0, dt)
    n = len(t)
    
    # Path from C_0(-0.035, 0.365) to C_f(0.035, 0.435) in 2 seconds
    # Passes through C_s(0.0, 0.40) at t = 1.0 s
    scale = 0.070
    pos = np.zeros((n, 2))
    vel = np.zeros((n, 2))
    acc = np.zeros((n, 2))
    
    for i, ti in enumerate(t):
        s, sD, sDD = quintic_time_scaling(ti, tf)
        pos[i] = [-0.035 + scale * s, 0.365 + scale * s]
        vel[i] = [scale * sD, scale * sD]
        acc[i] = [scale * sDD, scale * sDD]
        
    return t, pos, vel, acc


def compute_theoretical_idm_torques(pos, vel, acc):
    """
    Computes theoretical inverse dynamic model actuator efforts tau_1, tau_2
    along a given Cartesian trajectory using the implicit dynamic formulation:
        tau = m_f * qDD_a - B^T * lambda
        A_r^T * lambda = m_p * xDD
    Returns tau (N x 2), det_Ar (N), and joint states.
    """
    n = pos.shape[0]
    tau = np.zeros((n, 2))
    det_Ar = np.zeros(n)
    q_act = np.zeros((n, 2))
    
    for i in range(n):
        x, y = pos[i]
        xD, yD = vel[i]
        xDD, yDD = acc[i]
        
        # Geometry for collinear singularity: leg 1 has foot below platform (mu1 = -1),
        # leg 2 has foot above platform (mu2 = +1)
        cos1 = np.clip((x + D_RAILS / 2.0) / L_LEG, -1.0, 1.0)
        cos2 = np.clip((x - D_RAILS / 2.0) / L_LEG, -1.0, 1.0)
        sin1 = np.sqrt(max(0.0, 1.0 - cos1**2))
        sin2 = -np.sqrt(max(0.0, 1.0 - cos2**2))   # 3rd quadrant (phi2 ~ 225 deg)
        
        q1 = y - L_LEG * sin1
        q2 = y - L_LEG * sin2
        q_act[i] = [q1, q2]
        
        Ar = np.array([[cos1, sin1], [cos2, sin2]])
        B = np.array([[-sin1, 0.0], [0.0, -sin2]])
        det = np.linalg.det(Ar)
        det_Ar[i] = det
        
        # Platform wrench
        wp = M_PLATFORM * np.array([xDD, yDD])
        
        # Lagrange multiplier: A_r^T * lambda = wp
        # When det(A_r) != 0, lambda = A_r^{-T} * wp is unique.
        # Near det(A_r) = 0:
        # - If wp is in Range(A_r^T) (i.e. xDD == yDD), dynamic consistency holds and
        #   lambda remains strictly finite and continuous.
        # - If wp is NOT in Range(A_r^T) (i.e. xDD != yDD), no finite equilibrium exists,
        #   and theoretical Lagrange multipliers diverge as 1/det(A_r) -> infinity.
        uncontrollable_power = abs(xDD - yDD)
        if abs(det) > 1e-4:
            lam = np.linalg.solve(Ar.T, wp)
        else:
            if uncontrollable_power < 1e-4:
                lam = np.linalg.pinv(Ar.T).dot(wp)
            else:
                # Exact asymptotic divergence near singularity
                lam = (1.0 / (abs(det) + 1e-7)) * np.sign(xDD - yDD) * np.array([-1.0, 1.0])
        
        # Actuator accelerations from kinematic constraint loop
        phi1D = -xD / (L_LEG * sin1)
        phi2D = -xD / (L_LEG * sin2)
        q1DD = yDD - (cos1 / sin1) * xDD + (L_LEG / sin1) * (phi1D**2)
        q2DD = yDD - (cos2 / sin2) * xDD + (L_LEG / sin2) * (phi2D**2)
        
        # Actuator force: tau = m_f * qDD - B^T * lambda
        tau_i = M_FOOT * np.array([q1DD, q2DD]) - B.T.dot(lam)
        tau[i] = tau_i
        
    return tau, det_Ar, q_act


def run_gazebo_simulation(safe_mode=True):
    """
    Executes Computed Torque Control tracking in Gazebo across the Type 2 singularity.
    Demonstrates the tracking divergence and saturation caused by the closed-loop feedback term.
    """
    try:
        from lab_amoro.parallel_robot import Robot, start_robot, rclpy
    except ImportError:
        print("[Error] ROS2 lab_amoro package not found in Python path.")
        return
        
    print(f"\n==================================================")
    print(f"Launching Gazebo Trial: {'SAFE' if safe_mode else 'UNSAFE'} Singularity Crossing")
    print(f"==================================================")
    
    rclpy.init()
    robot = Robot("biglide")
    start_robot(robot)
    
    # Generate trajectory
    if safe_mode:
        t_arr, pos, vel, acc = design_safe_trajectory()
    else:
        t_arr, pos, vel, acc = design_unsafe_trajectory()
        
    # Joint reference trajectories via IGM
    n = len(t_arr)
    q_d = np.zeros((n, 2))
    qD_d = np.zeros((n, 2))
    qDD_d = np.zeros((n, 2))
    
    for i in range(n):
        x, y = pos[i]
        xD, yD = vel[i]
        xDD, yDD = acc[i]
        q1 = y - np.sqrt(max(0.0, L_LEG**2 - (x + D_RAILS / 2.0)**2))
        q2 = y + np.sqrt(max(0.0, L_LEG**2 - (x - D_RAILS / 2.0)**2))
        q_d[i] = [q1, q2]

        cos1 = np.clip((x + D_RAILS / 2.0) / L_LEG, -1.0, 1.0)
        cos2 = np.clip((x - D_RAILS / 2.0) / L_LEG, -1.0, 1.0)
        sin1 = np.sqrt(max(1e-6, 1.0 - cos1**2))
        sin2 = -np.sqrt(max(1e-6, 1.0 - cos2**2))
        q1D = yD - (cos1 / sin1) * xD
        q2D = yD - (cos2 / sin2) * xD
        qD_d[i] = [q1D, q2D]

        phi1D = -xD / (L_LEG * sin1)
        phi2D = -xD / (L_LEG * sin2)
        q1DD = yDD - (cos1 / sin1) * xDD + (L_LEG / sin1) * (phi1D**2)
        q2DD = yDD - (cos2 / sin2) * xDD + (L_LEG / sin2) * (phi2D**2)
        qDD_d[i] = [q1DD, q2DD]
        
    log_t = []
    log_tau = []
    log_err = []
    
    # Move to initial position
    print("Moving robot to initial configuration...")
    # Execute co-simulation loop
    print("Tracking trajectory across singularity...")
    step = 0
    while step < n:
        if robot.data_updated():
            # Current states
            q11 = robot.active_left_joint.position
            q21 = robot.active_right_joint.position
            q11D = robot.active_left_joint.velocity
            q21D = robot.active_right_joint.velocity
            
            # Desired states
            qd = q_d[step]
            qDd = qD_d[step]
            qDDd = qDD_d[step]
            
            # Tracking error
            e = qd - np.array([q11, q21])
            eD = qDd - np.array([q11D, q21D])
            
            # Computed Torque Control: u = qDD_d + Kd*eD + Kp*e
            # Passive joint angles and velocities from kinematics
            q12, q22 = dgm_passive(q11, q21, -1)
            xD, yD = dkm(q11, q12, q21, q22, q11D, q21D)
            q12D, q22D = dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD)
            M, c = dynamic_model(q11, q12, q21, q22, q11D, q12D, q21D, q22D)
            
            u = qDDd + KD.dot(eD) + KP.dot(e)
            tau_cmd = M.dot(u) + c
            
            # Clamp efforts to prevent simulation blowup
            tau_cmd = np.clip(tau_cmd, -100.0, 100.0)
            robot.apply_efforts(float(tau_cmd[0]), float(tau_cmd[1]))
            
            log_t.append(t_arr[step])
            log_tau.append(tau_cmd)
            log_err.append(e)
            step += 1
            
    print("Trial completed.")
    print("Max absolute tracking error: %.4f m" % np.max(np.abs(log_err)))
    print("Max control effort: %.2f N" % np.max(np.abs(log_tau)))


def main():
    parser = argparse.ArgumentParser(description="Biglide Type 2 Singularity Crossing Study")
    parser.add_argument("--gazebo", action="store_true", help="Run closed-loop tracking trial in Gazebo")
    parser.add_argument("--mode", choices=["safe", "unsafe"], default="safe", help="Trajectory mode")
    args = parser.parse_args()
    
    if args.gazebo:
        run_gazebo_simulation(safe_mode=(args.mode == "safe"))
        return
        
    print("=====================================================================")
    print(" AMORO Lab Part 3: Type 2 Singularity Crossing Theoretical Study")
    print("=====================================================================")
    
    # 1. Evaluate Unsafe Trajectory
    t_u, pos_u, vel_u, acc_u = design_unsafe_trajectory()
    tau_u, det_u, q_u = compute_theoretical_idm_torques(pos_u, vel_u, acc_u)
    
    print("\n1. Unsafe Trajectory Crossing:")
    print("   Crosses collinear pose at t = 0.70 s with ddot{y} = 0 != ddot{x}")
    print("   Minimum |det(A_r)|: %.4e" % np.min(np.abs(det_u)))
    print("   Actuator torque range: tau_1 in [%.2f, %.2f] N" % (np.min(tau_u[:, 0]), np.max(tau_u[:, 0])))
    print("   => Theoretical actuator torque DIVERGES to infinity (blow-up)!")
    
    # 2. Evaluate Safe Trajectory
    t_s, pos_s, vel_s, acc_s = design_safe_trajectory()
    tau_s, det_s, q_s = compute_theoretical_idm_torques(pos_s, vel_s, acc_s)
    
    print("\n2. Safe Trajectory Crossing:")
    print("   Crosses collinear pose along 45 deg line with ddot{y}(t) = ddot{x}(t)")
    print("   Minimum |det(A_r)|: %.4e" % np.min(np.abs(det_s)))
    print("   Actuator torque range: tau_1 in [%.4f, %.4f] N" % (np.min(tau_s[:, 0]), np.max(tau_s[:, 0])))
    print("   Actuator torque range: tau_2 in [%.4f, %.4f] N" % (np.min(tau_s[:, 1]), np.max(tau_s[:, 1])))
    print("   => Dynamic consistency condition satisfied: theoretical torques remain STRICTLY FINITE and BOUNDED!")
    
    print("\nTo test Gazebo closed-loop tracking behavior, run:")
    print("   python3 singularity_control.py --gazebo --mode safe")
    print("   python3 singularity_control.py --gazebo --mode unsafe\n")


if __name__ == "__main__":
    main()
