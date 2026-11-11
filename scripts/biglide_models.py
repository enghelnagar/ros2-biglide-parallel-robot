from math import *
import numpy as np

# Geometric parameters for Biglide
l = 0.2828427
d = 0.4
# Dynamic parameters
mp = 3.0
mf = 1.0

# -----------------------------------------------------------------------------
# Conventions (consistent with the Gazebo model and the provided test scripts)
#   - O is the middle of [A1(0) A2(0)], rails at x = -d/2 (left) and x = +d/2 (right)
#   - q11, q21 : positions of the left / right feet along y0 (actuated prismatic joints)
#   - q12, q22 : ABSOLUTE angles of the legs A1C, A2C w.r.t. x0 (phi_1, phi_2)
#                (feet only translate, so the relative angle measured by Gazebo
#                 is also the absolute angle of the leg)
#   - assembly mode / working modes: -1 <=> end-effector C ABOVE the feet
#     (this is the Gazebo configuration and the value used in model_test.py,
#      inverse_model_test.py and model_eval.py)
# -----------------------------------------------------------------------------


def dgm(q11, q21, assembly_mode):
    """
    Direct Geometric Model (DGM)
    Calculates end-effector coordinates (x, y) given active joint positions (q11, q21).
    assembly_mode = -1 : C above the line A1A2 (Gazebo configuration)
    assembly_mode = +1 : C below the line A1A2
    """
    A = (q11 - q21) / d
    B = -A * (q11 + q21) / 2.0

    a_y = 1 + A**2
    b_y = 2 * (A * (B + d/2.0) - q11)
    c_y = (B + d/2.0)**2 + q11**2 - l**2

    discriminant = b_y**2 - 4 * a_y * c_y

    if discriminant < 0:
        # Point out of the workspace
        return float('nan'), float('nan')

    y = (-b_y - assembly_mode * sqrt(discriminant)) / (2 * a_y)
    x = A * y + B

    return x, y


def igm(x, y, gamma1, gamma2):
    """
    Inverse Geometric Model (IGM)
    Calculates active joint positions (q11, q21) given end-effector coordinates (x, y).
    gamma_i = -1 : foot i below C (Gazebo configuration)
    gamma_i = +1 : foot i above C
    """
    q11 = y + gamma1 * sqrt(l**2 - (x + d/2.0)**2)
    q21 = y + gamma2 * sqrt(l**2 - (x - d/2.0)**2)
    return q11, q21


def dgm_passive(q11, q21, assembly_mode):
    """
    Calculates passive joint angles (q12, q22) given active joint positions.
    """
    x, y = dgm(q11, q21, assembly_mode)

    q12 = atan2(y - q11, x + d/2.0)
    q22 = atan2(y - q21, x - d/2.0)
    return q12, q22


def compute_A_B(q11, q12, q21, q22):
    """
    Kinematic constraint matrices:  A_r * xiD + B * qaD = 0
        A_r = [u12^T ; u22^T]   with u_i2 = [cos(q_i2), sin(q_i2)]
        B   = -diag(sin(q12), sin(q22))
    """
    A_r = np.array([
        [cos(q12), sin(q12)],
        [cos(q22), sin(q22)]
    ])

    B_mat = -np.array([
        [sin(q12), 0.0],
        [0.0, sin(q22)]
    ])

    return A_r, B_mat


def compute_convective_terms(q12, q22, q12D, q22D):
    """
    Convective vector b_p of the 2nd order model:  A_r*xiDD + B*qaDD + b_p = 0
    """
    b_p = np.array([
        l * q12D**2,
        l * q22D**2
    ])
    return b_p


def dkm(q11, q12, q21, q22, q11D, q21D):
    """
    Direct Kinematic Model - 1st Order
    xiD = J * qaD,   J = -A_r^-1 B
    """
    A_r, B_mat = compute_A_B(q11, q12, q21, q22)
    J = -np.linalg.solve(A_r, B_mat)
    v = J.dot(np.array([q11D, q21D]))
    xD, yD = v[0], v[1]
    return xD, yD


def ikm(q11, q12, q21, q22, xD, yD):
    """
    Inverse Kinematic Model - 1st Order
    qaD = J^-1 * xiD,   J^-1 = -B^-1 A_r
    """
    A_r, B_mat = compute_A_B(q11, q12, q21, q22)
    J_inv = -np.linalg.solve(B_mat, A_r)
    qD_a = J_inv.dot(np.array([xD, yD]))
    q11D, q21D = qD_a[0], qD_a[1]
    return q11D, q21D


def dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD):
    """
    Passive joint velocities (projection of the leg velocity equations on v_i2).
    """
    q12D = (-sin(q12) * xD + cos(q12) * (yD - q11D)) / l
    q22D = (-sin(q22) * xD + cos(q22) * (yD - q21D)) / l
    return q12D, q22D


def dkm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD):
    """
    Direct Kinematic Model - 2nd Order
    xiDD = J * qaDD + a_t,   a_t = -A_r^-1 b_p
    """
    A_r, B_mat = compute_A_B(q11, q12, q21, q22)
    b_p = compute_convective_terms(q12, q22, q12D, q22D)
    rhs = -B_mat.dot(np.array([q11DD, q21DD])) - b_p
    accel = np.linalg.solve(A_r, rhs)
    xDD, yDD = accel[0], accel[1]
    return xDD, yDD


def ikm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, xDD, yDD):
    """
    Inverse Kinematic Model - 2nd Order
    qaDD = J^-1 * xiDD + a_q,   a_q = -B^-1 b_p
    """
    A_r, B_mat = compute_A_B(q11, q12, q21, q22)
    b_p = compute_convective_terms(q12, q22, q12D, q22D)
    rhs = -A_r.dot(np.array([xDD, yDD])) - b_p
    qDD_a = np.linalg.solve(B_mat, rhs)
    q11DD, q21DD = qDD_a[0], qDD_a[1]
    return q11DD, q21DD


def dkm2_passive(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD, xDD, yDD):
    """
    Passive joint accelerations (projection of the leg acceleration equations on v_i2).
    """
    q12DD = (-sin(q12) * xDD + cos(q12) * (yDD - q11DD)) / l
    q22DD = (-sin(q22) * xDD + cos(q22) * (yDD - q21DD)) / l
    return q12DD, q22DD


def dynamic_model(q11, q12, q21, q22, q11D, q12D, q21D, q22D):
    """
    Inverse Dynamic Model:  tau = M * qaDD + c

    Virtual tree: the two feet (mass mf each, translation along y0) and the free
    platform (lumped mass mp at C; in Gazebo the masses of the two legs are located
    at C, hence mp = 1 + 1 + 1 = 3 kg).
        tau = tau_t + J^T w_p,   tau_t = mf * qaDD,   w_p = mp * xiDD
        xiDD = J qaDD + a_t
    =>  M = mf*I + mp*J^T J ,   c = mp*J^T a_t
    The mechanism moves in the horizontal plane (gravity along -z0),
    so there is NO gravity term.
    """
    A_r, B_mat = compute_A_B(q11, q12, q21, q22)
    b_p = compute_convective_terms(q12, q22, q12D, q22D)

    J = -np.linalg.solve(A_r, B_mat)      # direct Jacobian
    a_t = -np.linalg.solve(A_r, b_p)      # Cartesian convective acceleration

    M = mf * np.eye(2) + mp * J.T.dot(J)
    c = mp * J.T.dot(a_t)

    return M, c
