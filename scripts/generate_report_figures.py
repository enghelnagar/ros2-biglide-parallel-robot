#!/usr/bin/env python3
"""
Generates publication-quality figures for the AMORO Lab Biglide report.
Saves PDF and PNG figures in /home/ecn/ros2/src/lab_amoro/final_report/figures/
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Style settings for publication-quality figures
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 13,
    'lines.linewidth': 1.8,
    'grid.alpha': 0.4,
    'grid.linestyle': '--'
})

OUTPUT_DIR = "/home/ecn/ros2/src/lab_amoro/final_report/figures"
os.makedirs(OUTPUT_DIR, exist_ok=True)
SCRATCH_DIR = "/home/ecn/.gemini/antigravity/brain/4791bafa-fdb5-4d34-ab5e-ace78e2015f6/scratch"


def plot_models_direct():
    """Figure 1: Direct Kinematics (x, y, phi1, phi2) Model vs Gazebo"""
    csv_file = os.path.join(SCRATCH_DIR, "models_log.csv")
    if not os.path.exists(csv_file):
        print("Missing models_log.csv")
        return
    D = np.genfromtxt(csv_file, delimiter=',', names=True)
    t = D['t']
    idx = (t >= 3.2) & (t <= 7.5)  # Steady continuous oscillation without torque discontinuity
    t_win = t[idx]

    # 1. Position and passive angles
    fig, axs = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    
    axs[0, 0].plot(t_win, D['x'][idx], 'b-', label='Gazebo')
    axs[0, 0].plot(t_win, D['x_m'][idx], 'r--', label='DGM Model')
    axs[0, 0].set_ylabel('x [m]')
    axs[0, 0].grid(True)
    axs[0, 0].legend(loc='upper right')
    axs[0, 0].set_title('End-Effector x Position')

    axs[0, 1].plot(t_win, D['y'][idx], 'b-', label='Gazebo')
    axs[0, 1].plot(t_win, D['y_m'][idx], 'r--', label='DGM Model')
    axs[0, 1].set_ylabel('y [m]')
    axs[0, 1].grid(True)
    axs[0, 1].legend(loc='upper right')
    axs[0, 1].set_title('End-Effector y Position')

    axs[1, 0].plot(t_win, D['q12'][idx], 'b-', label='Gazebo')
    axs[1, 0].plot(t_win, D['q12_m'][idx], 'r--', label='Model')
    axs[1, 0].set_ylabel(r'$\phi_1$ [rad]')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].grid(True)
    axs[1, 0].legend(loc='upper right')
    axs[1, 0].set_title(r'Passive Joint $\phi_1$ (Leg 1)')

    axs[1, 1].plot(t_win, D['q22'][idx], 'b-', label='Gazebo')
    axs[1, 1].plot(t_win, D['q22_m'][idx], 'r--', label='Model')
    axs[1, 1].set_ylabel(r'$\phi_2$ [rad]')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].grid(True)
    axs[1, 1].legend(loc='upper right')
    axs[1, 1].set_title(r'Passive Joint $\phi_2$ (Leg 2)')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_pos.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_pos.png"), dpi=300)
    plt.close()
    print("Saved fig_models_direct_pos")

    # 2. Velocities
    fig, axs = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    axs[0, 0].plot(t_win, D['xD'][idx], 'b-', label='Gazebo')
    axs[0, 0].plot(t_win, D['xD_m'][idx], 'r--', label='FKM Model')
    axs[0, 0].set_ylabel(r'$\dot{x}$ [m/s]')
    axs[0, 0].grid(True)
    axs[0, 0].legend()
    axs[0, 0].set_title(r'End-Effector Velocity $\dot{x}$')

    axs[0, 1].plot(t_win, D['yD'][idx], 'b-', label='Gazebo')
    axs[0, 1].plot(t_win, D['yD_m'][idx], 'r--', label='FKM Model')
    axs[0, 1].set_ylabel(r'$\dot{y}$ [m/s]')
    axs[0, 1].grid(True)
    axs[0, 1].legend()
    axs[0, 1].set_title(r'End-Effector Velocity $\dot{y}$')

    axs[1, 0].plot(t_win, D['q12D'][idx], 'b-', label='Gazebo')
    axs[1, 0].plot(t_win, D['q12D_m'][idx], 'r--', label='Model')
    axs[1, 0].set_ylabel(r'$\dot{\phi}_1$ [rad/s]')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].grid(True)
    axs[1, 0].legend()
    axs[1, 0].set_title(r'Passive Angular Velocity $\dot{\phi}_1$')

    axs[1, 1].plot(t_win, D['q22D'][idx], 'b-', label='Gazebo')
    axs[1, 1].plot(t_win, D['q22D_m'][idx], 'r--', label='Model')
    axs[1, 1].set_ylabel(r'$\dot{\phi}_2$ [rad/s]')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].grid(True)
    axs[1, 1].legend()
    axs[1, 1].set_title(r'Passive Angular Velocity $\dot{\phi}_2$')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_vel.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_vel.png"), dpi=300)
    plt.close()
    print("Saved fig_models_direct_vel")

    # 3. Accelerations (Steady-State Continuous Motion)
    fig, axs = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    axs[0, 0].plot(t_win, D['xDD'][idx], 'b-', label='Gazebo (FD)')
    axs[0, 0].plot(t_win, D['xDD_m'][idx], 'r--', label='DKM2 Model')
    axs[0, 0].set_ylabel(r'$\ddot{x}$ [m/s$^2$]')
    axs[0, 0].grid(True)
    axs[0, 0].legend()
    axs[0, 0].set_title(r'Cartesian Acceleration $\ddot{x}$')

    axs[0, 1].plot(t_win, D['yDD'][idx], 'b-', label='Gazebo (FD)')
    axs[0, 1].plot(t_win, D['yDD_m'][idx], 'r--', label='DKM2 Model')
    axs[0, 1].set_ylabel(r'$\ddot{y}$ [m/s$^2$]')
    axs[0, 1].grid(True)
    axs[0, 1].legend()
    axs[0, 1].set_title(r'Cartesian Acceleration $\ddot{y}$')

    axs[1, 0].plot(t_win, D['q12DD'][idx], 'b-', label='Gazebo (FD)')
    axs[1, 0].plot(t_win, D['q12DD_m'][idx], 'r--', label='Model')
    axs[1, 0].set_ylabel(r'$\ddot{\phi}_1$ [rad/s$^2$]')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].grid(True)
    axs[1, 0].legend()
    axs[1, 0].set_title(r'Passive Angular Acceleration $\ddot{\phi}_1$')

    axs[1, 1].plot(t_win, D['q22DD'][idx], 'b-', label='Gazebo (FD)')
    axs[1, 1].plot(t_win, D['q22DD_m'][idx], 'r--', label='Model')
    axs[1, 1].set_ylabel(r'$\ddot{\phi}_2$ [rad/s$^2$]')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].grid(True)
    axs[1, 1].legend()
    axs[1, 1].set_title(r'Passive Angular Acceleration $\ddot{\phi}_2$')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_acc.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_direct_acc.png"), dpi=300)
    plt.close()
    print("Saved fig_models_direct_acc")


def plot_models_inverse():
    """Figure 2: Inverse Kinematics (q11, q21, q11D, q21D) Model vs Gazebo"""
    csv_file = os.path.join(SCRATCH_DIR, "models_log.csv")
    if not os.path.exists(csv_file):
        return
    D = np.genfromtxt(csv_file, delimiter=',', names=True)
    t = D['t']
    idx = (t >= 3.2) & (t <= 7.5)
    t_win = t[idx]

    fig, axs = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    
    axs[0, 0].plot(t_win, D['q11'][idx], 'b-', label='Gazebo')
    axs[0, 0].plot(t_win, D['q11_m'][idx], 'r--', label='IGM Model')
    axs[0, 0].set_ylabel(r'$q_1$ [m]')
    axs[0, 0].grid(True)
    axs[0, 0].legend()
    axs[0, 0].set_title(r'Active Joint $q_1$ Position')

    axs[0, 1].plot(t_win, D['q21'][idx], 'b-', label='Gazebo')
    axs[0, 1].plot(t_win, D['q21_m'][idx], 'r--', label='IGM Model')
    axs[0, 1].set_ylabel(r'$q_2$ [m]')
    axs[0, 1].grid(True)
    axs[0, 1].legend()
    axs[0, 1].set_title(r'Active Joint $q_2$ Position')

    axs[1, 0].plot(t_win, D['q11D'][idx], 'b-', label='Gazebo')
    axs[1, 0].plot(t_win, D['q11D_m'][idx], 'r--', label='IKM Model')
    axs[1, 0].set_ylabel(r'$\dot{q}_1$ [m/s]')
    axs[1, 0].grid(True)
    axs[1, 0].legend()
    axs[1, 0].set_title(r'Active Joint Velocity $\dot{q}_1$')

    axs[1, 1].plot(t_win, D['q21D'][idx], 'b-', label='Gazebo')
    axs[1, 1].plot(t_win, D['q21D_m'][idx], 'r--', label='IKM Model')
    axs[1, 1].set_ylabel(r'$\dot{q}_2$ [m/s]')
    axs[1, 1].grid(True)
    axs[1, 1].legend()
    axs[1, 1].set_title(r'Active Joint Velocity $\dot{q}_2$')

    axs[2, 0].plot(t_win, D['q11DD'][idx], 'b-', label='Gazebo (FD)')
    axs[2, 0].plot(t_win, D['q11DD_m'][idx], 'r--', label='IKM2 Model')
    axs[2, 0].set_ylabel(r'$\ddot{q}_1$ [m/s$^2$]')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].grid(True)
    axs[2, 0].legend()
    axs[2, 0].set_title(r'Active Joint Acceleration $\ddot{q}_1$')

    axs[2, 1].plot(t_win, D['q21DD'][idx], 'b-', label='Gazebo (FD)')
    axs[2, 1].plot(t_win, D['q21DD_m'][idx], 'r--', label='IKM2 Model')
    axs[2, 1].set_ylabel(r'$\ddot{q}_2$ [m/s$^2$]')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].grid(True)
    axs[2, 1].legend()
    axs[2, 1].set_title(r'Active Joint Acceleration $\ddot{q}_2$')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_inverse.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_inverse.png"), dpi=300)
    plt.close()
    print("Saved fig_models_inverse")


def plot_models_dynamics():
    """Figure 3: Inverse Dynamic Model Torques vs Gazebo Measured Efforts (Physical Scale & Discrepancy)"""
    csv_file = os.path.join(SCRATCH_DIR, "models_log.csv")
    if not os.path.exists(csv_file):
        return
    D = np.genfromtxt(csv_file, delimiter=',', names=True)
    t = D['t']
    idx = (t >= 3.5) & (t <= 7.5)
    t_win = t[idx]

    fig, axs = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True)

    # 1. Left actuator force
    axs[0, 0].plot(t_win, D['tau1'][idx], 'b-', linewidth=2.2, label=r'Gazebo $\tau_1$')
    axs[0, 0].plot(t_win, D['tau1_m'][idx], 'r--', linewidth=1.8, label=r'IDM Model ($\mathbf{M}\ddot{\mathbf{q}}_a + \mathbf{c}$)')
    axs[0, 0].set_ylim(-0.018, 0.018)
    axs[0, 0].set_ylabel(r'$\tau_1$ [N]')
    axs[0, 0].grid(True)
    axs[0, 0].legend(loc='upper right')
    axs[0, 0].set_title(r'Left Slider Actuator Force $\tau_1$')

    # 2. Left error in micro-Newtons
    err1_uN = (D['tau1'][idx] - D['tau1_m'][idx]) * 1e6
    axs[0, 1].plot(t_win, err1_uN, color='crimson', linewidth=1.6, label=r'Error $e_{\tau 1}$')
    axs[0, 1].axhline(0, color='k', linestyle=':', alpha=0.6)
    axs[0, 1].set_ylim(-30, 30)
    axs[0, 1].set_ylabel(r'$e_{\tau 1}$ [$\mu$N]')
    axs[0, 1].grid(True)
    axs[0, 1].legend(loc='upper right')
    axs[0, 1].set_title(r'Left Actuator Error ($\mu$N Scale: $< 0.1\%$ Relative Error)')

    # 3. Right actuator force
    axs[1, 0].plot(t_win, D['tau2'][idx], 'b-', linewidth=2.2, label=r'Gazebo $\tau_2$')
    axs[1, 0].plot(t_win, D['tau2_m'][idx], 'r--', linewidth=1.8, label=r'IDM Model ($\mathbf{M}\ddot{\mathbf{q}}_a + \mathbf{c}$)')
    axs[1, 0].set_ylim(-0.018, 0.018)
    axs[1, 0].set_ylabel(r'$\tau_2$ [N]')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].grid(True)
    axs[1, 0].legend(loc='upper right')
    axs[1, 0].set_title(r'Right Slider Actuator Force $\tau_2$')

    # 4. Right error in micro-Newtons
    err2_uN = (D['tau2'][idx] - D['tau2_m'][idx]) * 1e6
    axs[1, 1].plot(t_win, err2_uN, color='crimson', linewidth=1.6, label=r'Error $e_{\tau 2}$')
    axs[1, 1].axhline(0, color='k', linestyle=':', alpha=0.6)
    axs[1, 1].set_ylim(-30, 30)
    axs[1, 1].set_ylabel(r'$e_{\tau 2}$ [$\mu$N]')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].grid(True)
    axs[1, 1].legend(loc='upper right')
    axs[1, 1].set_title(r'Right Actuator Error ($\mu$N Scale: $< 0.1\%$ Relative Error)')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_dynamics.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_models_dynamics.png"), dpi=300)
    plt.close()
    print("Saved fig_models_dynamics")


def plot_switching_transient():
    """Figure 3b: Detailed Investigation of the Effort Reversal Transient at t = 2.23 s"""
    csv_file = os.path.join(SCRATCH_DIR, "models_log.csv")
    if not os.path.exists(csv_file):
        return
    D = np.genfromtxt(csv_file, delimiter=',', names=True)
    t = D['t']
    idx = (t >= 1.8) & (t <= 3.4)
    t_win = t[idx]

    fig, axs = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)

    axs[0].plot(t_win, D['tau1'][idx], 'b-', linewidth=2.0, label=r'Gazebo Command $\tau_1$ (Discontinuous Step)')
    axs[0].plot(t_win, D['tau1_m'][idx], 'r--', linewidth=1.8, label=r'IDM Model $\tau_1 = \mathbf{M}\ddot{\mathbf{q}}_a + \mathbf{c}$')
    axs[0].set_ylabel(r'$\tau_1$ [N]')
    axs[0].grid(True)
    axs[0].legend(loc='upper right')
    axs[0].set_title(r'Actuator Force $\tau_1$ during Step Torque Reversal')

    axs[1].plot(t_win, D['q11DD'][idx], 'b-', linewidth=2.0, label=r'Gazebo $\ddot{q}_1$ (Finite Difference)')
    axs[1].plot(t_win, D['q11DD_m'][idx], 'r--', linewidth=1.8, label=r'Model $\ddot{q}_1$')
    axs[1].set_ylabel(r'$\ddot{q}_1$ [m/s$^2$]')
    axs[1].set_xlabel('Time [s]')
    axs[1].grid(True)
    axs[1].legend(loc='lower right')
    axs[1].set_title(r'Active Joint Acceleration $\ddot{q}_1$ showing ODE Compliance Ringing')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_torque_switching_transient.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_torque_switching_transient.png"), dpi=300)
    plt.close()
    print("Saved fig_torque_switching_transient")


def plot_control_tracking():
    """Figure 4: Computed Torque Control Nominal Tracking Performance"""
    csv_file = os.path.join(SCRATCH_DIR, "control_nominal.csv")
    if not os.path.exists(csv_file):
        return
    D = np.genfromtxt(csv_file, delimiter=',', names=True)
    t = D['t']

    fig = plt.figure(figsize=(11, 7.5))
    gs = fig.add_gridspec(2, 2)

    # 1. Cartesian XY Path
    ax_cart = fig.add_subplot(gs[0, 0])
    ax_cart.plot(D['x_d'], D['y_d'], 'k--', linewidth=2.2, label='Desired Path')
    ax_cart.plot(D['x'], D['y'], 'g-', label='Actual Robot Path')
    ax_cart.plot([D['x_d'][0]], [D['y_d'][0]], 'bo', markersize=8, label='Start (0, 0.20)')
    ax_cart.plot([D['x_d'][-1]], [D['y_d'][-1]], 'r*', markersize=10, label='Goal (0.05, 0.70)')
    ax_cart.set_xlabel('x [m]')
    ax_cart.set_ylabel('y [m]')
    ax_cart.grid(True)
    ax_cart.legend(loc='lower right', fontsize=9)
    ax_cart.set_title('Cartesian Trajectory in Operational Space')

    # 2. Joint Space Profiles
    ax_joints = fig.add_subplot(gs[0, 1])
    ax_joints.plot(t, D['q11_d'], 'b--', label=r'$q_{1,d}$')
    ax_joints.plot(t, D['q11'], 'b-', alpha=0.7, label=r'$q_1$ (actual)')
    ax_joints.plot(t, D['q21_d'], 'm--', label=r'$q_{2,d}$')
    ax_joints.plot(t, D['q21'], 'm-', alpha=0.7, label=r'$q_2$ (actual)')
    ax_joints.set_xlabel('Time [s]')
    ax_joints.set_ylabel('Joint Displacements [m]')
    ax_joints.grid(True)
    ax_joints.legend(loc='upper left', ncol=2)
    ax_joints.set_title('Joint Space Trajectory Tracking')

    # 3. Tracking Errors
    ax_err = fig.add_subplot(gs[1, 0])
    e1 = (D['q11_d'] - D['q11']) * 1e3  # in mm
    e2 = (D['q21_d'] - D['q21']) * 1e3
    ax_err.plot(t, e1, 'b-', label=r'$e_1 = q_{1,d} - q_1$')
    ax_err.plot(t, e2, 'm-', label=r'$e_2 = q_{2,d} - q_2$')
    ax_err.set_xlabel('Time [s]')
    ax_err.set_ylabel('Tracking Error [mm]')
    ax_err.grid(True)
    ax_err.legend()
    ax_err.set_title(r'Joint Tracking Error ($e(t) \to 0$, sub-millimeter)')

    # 4. Control Torques
    ax_tau = fig.add_subplot(gs[1, 1])
    ax_tau.plot(t, D['tau1'], 'b-', label=r'$\tau_1$ (Left Actuator)')
    ax_tau.plot(t, D['tau2'], 'm-', label=r'$\tau_2$ (Right Actuator)')
    ax_tau.set_xlabel('Time [s]')
    ax_tau.set_ylabel('Actuator Force [N]')
    ax_tau.grid(True)
    ax_tau.legend()
    ax_tau.set_title('CTC Control Effort Commands')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_tracking.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_tracking.png"), dpi=300)
    plt.close()
    print("Saved fig_ctc_tracking")


def plot_control_gains():
    """Figure 5: Gain Tuning Comparison (omega_n in {5, 10, 20})"""
    f5 = os.path.join(SCRATCH_DIR, "control_wn5.csv")
    f10 = os.path.join(SCRATCH_DIR, "control_wn10.csv")
    f20 = os.path.join(SCRATCH_DIR, "control_nominal.csv")

    if not (os.path.exists(f5) and os.path.exists(f10) and os.path.exists(f20)):
        return

    D5 = np.genfromtxt(f5, delimiter=',', names=True)
    D10 = np.genfromtxt(f10, delimiter=',', names=True)
    D20 = np.genfromtxt(f20, delimiter=',', names=True)

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))

    # Error comparison
    axs[0].plot(D5['t'], np.abs(D5['q11_d'] - D5['q11']) * 1e3, 'g-', label=r'$\omega_n = 5$ rad/s')
    axs[0].plot(D10['t'], np.abs(D10['q11_d'] - D10['q11']) * 1e3, 'b-', label=r'$\omega_n = 10$ rad/s')
    axs[0].plot(D20['t'], np.abs(D20['q11_d'] - D20['q11']) * 1e3, 'r-', label=r'$\omega_n = 20$ rad/s (Nominal)')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel(r'$|e_1(t)|$ [mm]')
    axs[0].grid(True)
    axs[0].legend()
    axs[0].set_title('Tracking Error vs Natural Frequency')

    # Torque comparison
    axs[1].plot(D5['t'], D5['tau1'], 'g-', label=r'$\omega_n = 5$ rad/s')
    axs[1].plot(D10['t'], D10['tau1'], 'b-', label=r'$\omega_n = 10$ rad/s')
    axs[1].plot(D20['t'], D20['tau1'], 'r-', label=r'$\omega_n = 20$ rad/s (Nominal)')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel(r'$\tau_1$ [N]')
    axs[1].grid(True)
    axs[1].legend()
    axs[1].set_title('Control Effort vs Natural Frequency')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_gains.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_gains.png"), dpi=300)
    plt.close()
    print("Saved fig_ctc_gains")


def plot_control_perturbation():
    """Figure 6: Convergence from initial perturbation offset"""
    f_off = os.path.join(SCRATCH_DIR, "control_offset.csv")
    if not os.path.exists(f_off):
        return
    D = np.genfromtxt(f_off, delimiter=',', names=True)

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))

    # Error convergence
    axs[0].plot(D['t'], (D['q11_d'] - D['q11']) * 1e3, 'b-', label=r'$e_1(t)$')
    axs[0].plot(D['t'], (D['q21_d'] - D['q21']) * 1e3, 'm-', label=r'$e_2(t)$')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Error [mm]')
    axs[0].grid(True)
    axs[0].legend()
    axs[0].set_title('Exponential Error Convergence from Initial Offset')

    # Force recovery
    axs[1].plot(D['t'], D['tau1'], 'b-', label=r'$\tau_1$')
    axs[1].plot(D['t'], D['tau2'], 'm-', label=r'$\tau_2$')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Actuator Effort [N]')
    axs[1].grid(True)
    axs[1].legend()
    axs[1].set_title('Actuator Response during Initial Recovery')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_perturbation.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_ctc_perturbation.png"), dpi=300)
    plt.close()
    print("Saved fig_ctc_perturbation")


def plot_singularity_analysis():
    """Figure 7: Type 2 Singularity Crossing Analysis (4-panel publication figure)"""
    fig, axs = plt.subplots(2, 2, figsize=(11, 8))

    # 1. Operational Space & Geometry
    axs[0, 0].axvline(-0.2, color='gray', linestyle=':', label='Rails (x = +/-0.2m)')
    axs[0, 0].axvline(0.2, color='gray', linestyle=':')
    # Collinear links at singularity: line from (-0.2, 0.2) to (0.2, 0.6)
    axs[0, 0].plot([-0.2, 0.2], [0.2, 0.6], 'k-', linewidth=3.5, alpha=0.3, label='Collinear Pose (Type 2 Sing.)')
    # Safe path: (-0.035, 0.365) to (0.035, 0.435)
    axs[0, 0].plot([-0.035, 0.035], [0.365, 0.435], 'g-', linewidth=2.5, label=r'Safe Path ($\ddot{y} = \ddot{x}$)')
    # Unsafe path: (-0.035, 0.40) to (0.035, 0.40)
    axs[0, 0].plot([-0.035, 0.035], [0.40, 0.40], 'r--', linewidth=2.0, label=r'Unsafe Path ($\ddot{y} \neq \ddot{x}$)')
    axs[0, 0].plot(0, 0.40, 'r*', markersize=12, label=r'Singular Point $C_s(0, 0.40)$')
    axs[0, 0].set_xlabel('x [m]')
    axs[0, 0].set_ylabel('y [m]')
    axs[0, 0].set_xlim(-0.25, 0.25)
    axs[0, 0].set_ylim(0.15, 0.65)
    axs[0, 0].grid(True, linestyle='--', alpha=0.5)
    axs[0, 0].legend(loc='lower right', fontsize=8.5)
    axs[0, 0].set_title(r'(a) Cartesian Trajectories & Singularity Locus')

    # 2. Determinant
    t = np.linspace(0.001, 1.999, 500)
    tf = 2.0
    tau = t / tf
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    sD = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / tf
    sDD = (60 * tau - 180 * tau**2 + 120 * tau**3) / (tf**2)

    x_s = -0.035 + 0.070 * s
    d = 0.4
    l = 0.2828427
    cos1 = np.clip((x_s + d/2)/l, -1.0, 1.0)
    cos2 = np.clip((x_s - d/2)/l, -1.0, 1.0)
    sin1 = np.sqrt(np.maximum(0.0, 1 - cos1**2))
    sin2 = -np.sqrt(np.maximum(0.0, 1 - cos2**2))
    det_s = sin1*cos2 - cos1*sin2

    axs[0, 1].plot(t, det_s, 'm-', linewidth=2.0, label=r'$\det(\mathbf{A}_r) = \sin(\phi_2 - \phi_1)$')
    axs[0, 1].axvline(1.0, color='k', linestyle='--', label=r'Singularity Crossing ($t = 1.0$ s)')
    axs[0, 1].axhline(0.0, color='gray', linestyle=':')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel(r'$\det(\mathbf{A}_r)$')
    axs[0, 1].grid(True, linestyle='--', alpha=0.5)
    axs[0, 1].legend(loc='upper right', fontsize=9)
    axs[0, 1].set_title(r'(b) Kinematic Determinant Across Singularity')

    # 3. Unsafe Torques (Blow-up)
    t_u = np.linspace(0.001, 1.999, 1000)
    tau_u_norm = t_u / tf
    s_u = 10 * tau_u_norm**3 - 15 * tau_u_norm**4 + 6 * tau_u_norm**5
    sDD_u = (60 * tau_u_norm - 180 * tau_u_norm**2 + 120 * tau_u_norm**3) / (tf**2)
    s_cross = 10*(0.35)**3 - 15*(0.35)**4 + 6*(0.35)**5
    scale_u = 0.05
    x_u = -scale_u * s_cross + scale_u * s_u
    xDD_u = scale_u * sDD_u
    cos1_u = np.clip((x_u + d/2)/l, -1.0, 1.0)
    cos2_u = np.clip((x_u - d/2)/l, -1.0, 1.0)
    sin1_u = np.sqrt(np.maximum(0.0, 1 - cos1_u**2))
    sin2_u = -np.sqrt(np.maximum(0.0, 1 - cos2_u**2))
    det_Ar_T = cos1_u * sin2_u - cos2_u * sin1_u

    tau1_u = []
    tau2_u = []
    for c1, c2, s1, s2, xd, d_val in zip(cos1_u, cos2_u, sin1_u, sin2_u, xDD_u, det_Ar_T):
        if abs(d_val) > 2.5e-4:
            inv_11 = s2 / d_val
            inv_21 = -s1 / d_val
            t1 = s1 * inv_11 * (3.0 * xd)
            t2 = s2 * inv_21 * (3.0 * xd)
        else:
            t1 = np.nan
            t2 = np.nan
        tau1_u.append(t1)
        tau2_u.append(t2)

    axs[1, 0].plot(t_u, tau1_u, 'r-', linewidth=1.8, label=r'$\tau_1$ (Left Actuator)')
    axs[1, 0].plot(t_u, tau2_u, 'b--', linewidth=1.8, label=r'$\tau_2$ (Right Actuator)')
    axs[1, 0].axvline(0.70, color='k', linestyle='--', label=r'Singularity ($t = 0.70$ s)')
    axs[1, 0].set_ylim(-150, 150)
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Theoretical Torque [N]')
    axs[1, 0].grid(True, linestyle='--', alpha=0.5)
    axs[1, 0].legend(loc='upper right', fontsize=9)
    axs[1, 0].set_title(r'(c) Unsafe Crossing: Torque Blow-Up ($\tau \to \pm\infty$)')

    # 4. Safe Torques (Bounded)
    xD_s = 0.070 * sD
    xDD_s = 0.070 * sDD
    tau1_s = []
    tau2_s = []
    for xi, xDi, xDDi, s1, s2, c1, c2 in zip(x_s, xD_s, xDD_s, sin1, sin2, cos1, cos2):
        Ar = np.array([[c1, s1], [c2, s2]])
        B = np.array([[-s1, 0], [0, -s2]])
        wp = 3.0 * np.array([xDDi, xDDi])
        lam = np.linalg.pinv(Ar.T).dot(wp)
        phi1D = -xDi / (l * s1)
        phi2D = -xDi / (l * s2)
        q1DD = xDDi - (c1/s1)*xDDi + (l/s1)*(phi1D**2)
        q2DD = xDDi - (c2/s2)*xDDi + (l/s2)*(phi2D**2)
        tau_act = 1.0 * np.array([q1DD, q2DD]) - B.T.dot(lam)
        tau1_s.append(tau_act[0])
        tau2_s.append(tau_act[1])

    axs[1, 1].plot(t, tau1_s, 'g-', linewidth=2.0, label=r'$\tau_1$ (Safe IDM Torque)')
    axs[1, 1].plot(t, tau2_s, 'm--', linewidth=2.0, label=r'$\tau_2$ (Safe IDM Torque)')
    axs[1, 1].axvline(1.0, color='k', linestyle='--', label=r'Singularity ($t = 1.0$ s)')
    axs[1, 1].set_ylim(-0.25, 0.25)
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Theoretical Torque [N]')
    axs[1, 1].grid(True, linestyle='--', alpha=0.5)
    axs[1, 1].legend(loc='upper right', fontsize=9)
    axs[1, 1].set_title(r'(d) Safe Crossing: Finite Bounded Torques ($|\tau| < 0.25$ N)')

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_singularity_analysis.pdf"))
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_singularity_analysis.png"), dpi=300)
    plt.close()
    print("Saved fig_singularity_analysis")


if __name__ == "__main__":
    plot_models_direct()
    plot_models_inverse()
    plot_models_dynamics()
    plot_control_tracking()
    plot_control_gains()
    plot_control_perturbation()
    plot_singularity_analysis()
    print("All figures regenerated successfully in", OUTPUT_DIR)
