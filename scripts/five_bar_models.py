from math import *
import numpy as np

# Geometric parameters
l = 0.09
d = 0.118
# Dynamic parameters
ZZ1R = 1.0 * 0.045 * 0.045
ZZ2R = 1.0 * 0.045 * 0.045
mR = 0.5


def dgm(q11, q21, assembly_mode):

    oa21=np.array([d/2, 0])
    oa21_oa22=np.array([l*cos(q21), l*sin(q21)])
    a22_h=0.5*np.array([-l*cos(q21) -d+l*cos(q11),l*sin(q11)-l*sin(q21)])
    a=sqrt(a22_h[0]**2+a22_h[1]**2)

    h=sqrt(l**2-a**2)

    h_a13=assembly_mode*h/a*np.array([[0, -1], [1, 0]]) @ a22_h
    e=oa21+oa21_oa22+a22_h+h_a13
    x=e[0]
    y=e[1]
    return x, y


def igm(x, y, gamma1, gamma2):

    A1M1=0.5* np.array([x+d/2, y])
    c1= np.linalg.norm(A1M1)
    b1=sqrt(l**2-c1**2)
    HA13=gamma1*b1/c1*np.array([[0, -1], [1, 0]]) @ A1M1

    A11_A12= A1M1+HA13
    q11=atan2(A11_A12[1],A11_A12[0])


    A2M1=0.5* np.array([x-d/2, y])
    c2= np.linalg.norm(A2M1)
    b2=sqrt(l**2-c2**2)
    HA13=gamma2*b2/c2*np.array([[0, -1], [1, 0]]) @ A2M1

    A21_A22= A2M1+HA13
    q21=atan2(A21_A22[1],A21_A22[0])

    return q11, q21


def dgm_passive(q11, q21, assembly_mode):
    x,y=dgm(q11, q21, assembly_mode)
    q12=atan2(y/l-sin(q11),(x/l)+d/(2*l)-cos(q11))-q11
    q22=atan2(y/l-sin(q21),(x/l)-d/(2*l)-cos(q21))-q21

    return q12, q22


# You can create intermediate functions to avoid redundant code
def compute_A_B(q11, q12, q21, q22 ):

    u11=np.array([l*cos(q11)+d/2,l*sin(q11)])
    u21=np.array([l*cos(q21)-d/2,l*sin(q21)])

    u11=u11/np.linalg.norm(u11)
    u21=u21/np.linalg.norm(u21)


    u22=np.array([cos(q21+q22),sin(q21+q22)])
    u12=np.array([cos(q11+q12),sin(q11+q12)])

    v11=np.array([[0, -1], [1, 0]]) @ u11
    v21=np.array([[0, -1], [1, 0]]) @ u21


    A=np.vstack((u12.T,u22.T))
    B=np.array([[l*np.dot(u12,v11), 0], [0, l*np.dot(u22,v21)]])

    return A, B


def dkm(q11, q12, q21, q22, q11D, q21D):
    A,B=compute_A_B(q11, q12, q21, q22)
    D=np.linalg.inv (A) @ B @ np.array([q11D,q21D])
    xD = D[0]
    yD = D[1]
    return xD, yD


def ikm(q11, q12, q21, q22, xD, yD):

    A,B =compute_A_B(q11, q12, q21, q22)
    D=np.linalg.inv(B)@ A @ np.array([xD,yD])
    q11D = D[1]
    q21D = D[0]
    return q11D, q21D


def dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD):

    u11 = np.array([l * np.cos(q11) + d / 2, l * np.sin(q11)])
    u21 = np.array([l * np.cos(q21) - d / 2, l * np.sin(q21)])

    u11 = u11 / np.linalg.norm(u11)
    u21 = u21 / np.linalg.norm(u21)

    u22 = np.array([np.cos(q21 + q22), np.sin(q21 + q22)])
    u12 = np.array([np.cos(q11 + q12), np.sin(q11 + q12)])

    v11 = np.array([[0, 1], [-1, 0]]) @ u11
    v21 = np.array([[0, 1], [-1, 0]]) @ u21

    v12 = np.array([[0, 1], [-1, 0]]) @ u12
    v22 = np.array([[0, 1], [-1, 0]]) @ u22

    A = np.vstack((v12.T, v22.T))
    B = np.array([[l * np.dot(v12, v11) + l, 0],
                  [0, l * np.dot(v22, v21) + l]])


    C = np.array([xD, yD])


    qD_p = (1 / l) * (A @ C - B @ np.array([q11D, q21D]))

    q12D = qD_p[0]
    q22D = qD_p[1]

    return q12D, q22D


def dkm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD):

    u11 = np.array([l * np.cos(q11) + d / 2, l * np.sin(q11)])
    u21 = np.array([l * np.cos(q21) - d / 2, l * np.sin(q21)])

    u11 = u11 / np.linalg.norm(u11)
    u21 = u21 / np.linalg.norm(u21)

    u22 = np.array([np.cos(q21 + q22), np.sin(q21 + q22)])
    u12 = np.array([np.cos(q11 + q12), np.sin(q11 + q12)])

    v11 = np.array([[0, -1], [1, 0]]) @ u11
    v21 = np.array([[0, -1], [1, 0]]) @ u21

    v12 = np.array([[0, -1], [1, 0]]) @ u12
    v22 = np.array([[0, -1], [1, 0]]) @ u22

    A = np.vstack((u12.T,u22.T))
    B = np.array([[l*np.dot(u12,v11), 0], [0 , l*np.dot(u22,v21)]])
    D = np.array([[-l*q11D**2*np.dot(u12, u11)-l*(q11D+q12D)**2],[-l*q21D**2*np.dot(u22, u21)-l*(q21D+q22D)**2]])

    xsi_DD= np.linalg.inv(A) @ (B @ np.array([[q11DD],[q21DD]])+D)

    xDD = xsi_DD[0]
    yDD = xsi_DD[1]
    return xDD, yDD


def ikm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, xDD, yDD):

    u11 = np.array([l * np.cos(q11) + d / 2, l * np.sin(q11)])
    u21 = np.array([l * np.cos(q21) - d / 2, l * np.sin(q21)])

    u11 = u11 / np.linalg.norm(u11)
    u21 = u21 / np.linalg.norm(u21)

    u22 = np.array([np.cos(q21 + q22), np.sin(q21 + q22)])
    u12 = np.array([np.cos(q11 + q12), np.sin(q11 + q12)])

    v11 = np.array([[0, -1], [1, 0]]) @ u11
    v21 = np.array([[0, -1], [1, 0]]) @ u21

    v12 = np.array([[0, -1], [1, 0]]) @ u12
    v22 = np.array([[0, -1], [1, 0]]) @ u22

    A = np.vstack((u12.T,u22.T))
    B = np.array([[l*np.dot(u12,v11), 0], [0 , l*np.dot(u22,v21)]])
    D = np.array([[-l*q11D**2*np.dot(u12, u11)-l*(q11D+q12D)**2],[-l*q21D**2*np.dot(u22, u21)-l*(q21D+q22D)**2]])

    q_DD = np.linalg.inv(B) @ (A @ np.array([[xDD],[yDD]])-D)


    q11DD = q_DD[0]
    q21DD = q_DD[1]
    return q11DD, q21DD


def dkm2_passive(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD, xDD, yDD):

    u11 = np.array([l * np.cos(q11) + d / 2, l * np.sin(q11)])
    u21 = np.array([l * np.cos(q21) - d / 2, l * np.sin(q21)])

    u11 = u11 / np.linalg.norm(u11)
    u21 = u21 / np.linalg.norm(u21)

    u22 = np.array([np.cos(q21 + q22), np.sin(q21 + q22)])
    u12 = np.array([np.cos(q11 + q12), np.sin(q11 + q12)])

    v11 = np.array([[0, -1], [1, 0]]) @ u11
    v21 = np.array([[0, -1], [1, 0]]) @ u21

    v12 = np.array([[0, -1], [1, 0]]) @ u12
    v22 = np.array([[0, -1], [1, 0]]) @ u22

    A = np.vstack((v12.T, v22.T))
    B = np.array([[l * np.dot(v12, v11) + l, 0],
                  [0, l * np.dot(v22, v21) + l]])


    D = np.array([[-l*q11D**2 * np.dot(v12,u11)],[-l*q21D**2 * np.dot(v22,u21)]])
    print("shape is ;",A.shape)
    print("vector shape:", np.array([xDD, yDD]).shape)

    qD_p = (1 / l) * (
        A @ np.array([xDD, yDD])
        - B @ np.array([[q11DD], [q21DD]])
        - D
    )


    q12DD = qD_p[0]
    q22DD = qD_p[1]

    return q12DD, q22DD


def dynamic_model(q11, q12, q21, q22, q11D, q12D, q21D, q22D):
    M = np.zeros((2,2))
    c = 0
    return M, c
