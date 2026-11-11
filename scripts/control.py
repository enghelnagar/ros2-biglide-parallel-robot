from lab_amoro.parallel_robot import *
from lab_amoro.plot_tools import *
from biglide_models import *  # Biglide models
import sys
import numpy as np

# Working / assembly modes of the Gazebo Biglide (end-effector above the feet)
ASSEMBLY_MODE = -1
WORKING_MODE_1 = -1
WORKING_MODE_2 = -1

# Trajectory parameters (Part 3 of the lab)
DT = 0.01               # sampling time of the trajectory [s] (= simulation step)
T_FINAL = 2.0           # duration of the motion [s]
X_FINAL = 0.05          # final end-effector position [m]
Y_FINAL = 0.7

# PD gains of the computed torque control (critically damped, omega_n = 20 rad/s)
OMEGA_N = 20.0
ZETA = 1.0
KP = OMEGA_N**2 * np.eye(2)
KD = 2.0 * ZETA * OMEGA_N * np.eye(2)


def cartesian_trajectory(x_a, y_a, x_b, y_b, tf, dt=DT):
    """
    Fifth order polynomial trajectory from A to B with zero velocity and
    acceleration at both ends, sampled with dt.
    Returns time, position (N x 2), velocity (N x 2), acceleration (N x 2).
    """
    t = np.arange(0.0, tf + dt / 2.0, dt)
    tau = t / tf
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    sD = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / tf
    sDD = (60 * tau - 180 * tau**2 + 120 * tau**3) / tf**2
    delta = np.array([x_b - x_a, y_b - y_a])
    pos = np.array([x_a, y_a]) + np.outer(s, delta)
    vel = np.outer(sD, delta)
    acc = np.outer(sDD, delta)
    return t, pos, vel, acc


def joint_trajectory(pos, vel, acc):
    """
    Joint space trajectory (active joints) from the Cartesian trajectory
    using the inverse geometric, kinematic and second order kinematic models.
    """
    n = pos.shape[0]
    q = np.zeros((n, 2))
    qD = np.zeros((n, 2))
    qDD = np.zeros((n, 2))
    for i in range(n):
        x, y = pos[i]
        xD, yD = vel[i]
        xDD, yDD = acc[i]
        q11, q21 = igm(x, y, WORKING_MODE_1, WORKING_MODE_2)
        # passive joints from the geometry of the legs
        q12 = atan2(y - q11, x + d / 2.0)
        q22 = atan2(y - q21, x - d / 2.0)
        q11D, q21D = ikm(q11, q12, q21, q22, xD, yD)
        q12D, q22D = dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD)
        q11DD, q21DD = ikm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, xDD, yDD)
        q[i] = [q11, q21]
        qD[i] = [q11D, q21D]
        qDD[i] = [q11DD, q21DD]
    return q, qD, qDD


def computed_torque(q11, q21, q11D, q21D, q_t, qD_t, qDD_t, kp=KP, kd=KD):
    """
    Computed torque control law:
        alpha = qDD_t + Kd (qD_t - qD) + Kp (q_t - q)
        tau   = M alpha + c
    The passive joints are estimated from the active joints (only the
    actuated joints are measured on a real robot).
    """
    q12, q22 = dgm_passive(q11, q21, ASSEMBLY_MODE)
    xD, yD = dkm(q11, q12, q21, q22, q11D, q21D)
    q12D, q22D = dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD)
    M, c = dynamic_model(q11, q12, q21, q22, q11D, q12D, q21D, q22D)
    q = np.array([q11, q21])
    qD = np.array([q11D, q21D])
    alpha = qDD_t + kd.dot(qD_t - qD) + kp.dot(q_t - q)
    tau = M.dot(alpha) + c
    return tau


def main(args=None):
    # Initialize and start the ROS2 robot interface
    rclpy.init(args=args)
    robot = Robot("biglide")
    start_robot(robot)

    # Prepare plots
    app = QtGui.QApplication([])
    scope_joint1 = Scope("Joint 1 (q11) [m]", -0.1, 0.6)
    scope_joint2 = Scope("Joint 2 (q21) [m]", -0.1, 0.6)

    # Create the trajectory as arrays in Cartesian space (position, velocity, acceleration)
    # Initial position of the robot: q11 = q21 = 0  ->  (x, y) = (0, 0.2)
    x_0, y_0 = dgm(0.0, 0.0, ASSEMBLY_MODE)
    t_traj, pos, vel, acc = cartesian_trajectory(x_0, y_0, X_FINAL, Y_FINAL, T_FINAL)

    # Create the trajectory as arrays in joint space using the inverse models (position, velocity, acceleration)
    q_traj, qD_traj, qDD_traj = joint_trajectory(pos, vel, acc)
    trajectory = q_traj

    index = 0
    errors = []
    results_printed = False

    # Controller
    try:
        robot.apply_efforts(0.0, 0.0)  # Required to start the simulation
        while True:
            if robot.data_updated():
                # Robot available data - This is the only data that you can get from a real robot (joint encoders)
                q11 = robot.active_left_joint.position
                q21 = robot.active_right_joint.position
                q11D = robot.active_left_joint.velocity
                q21D = robot.active_right_joint.velocity

                # Desired state (the last point is held at the end of the trajectory)
                q_t = q_traj[index]
                qD_t = qD_traj[index]
                qDD_t = qDD_traj[index]

                # CTC controller
                tau = computed_torque(q11, q21, q11D, q21D, q_t, qD_t, qDD_t)
                tau_left, tau_right = tau[0], tau[1]
                robot.apply_efforts(tau_left, tau_right)

                # Scope update
                time = robot.get_time()
                if time < T_FINAL + 1.0:
                    scope_joint1.update(time, q11, q_t[0])
                    scope_joint2.update(time, q21, q_t[1])
                    errors.append([abs(q_t[0] - q11), abs(q_t[1] - q21)])
                elif not results_printed:
                    results_printed = True
                    err = np.array(errors)
                    x, y = dgm(q11, q21, ASSEMBLY_MODE)
                    print("Tracking error q11: mean = %.3e m, max = %.3e m" % (err[:, 0].mean(), err[:, 0].max()))
                    print("Tracking error q21: mean = %.3e m, max = %.3e m" % (err[:, 1].mean(), err[:, 1].max()))
                    print("Final end-effector position: x = %.4f m, y = %.4f m (target %.2f, %.2f)"
                          % (x, y, X_FINAL, Y_FINAL))

                if index < len(trajectory) - 1:
                    index += 1  # Next point in trajectory

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(sys.argv)
