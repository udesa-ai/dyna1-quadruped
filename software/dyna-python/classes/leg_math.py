from math import atan2, sqrt, sin, cos, pi
import numpy as np


def inv_k(leg, pos):
    """
    inv_k does the inverse kinematics of a Leg object. Equations taken from:
    https://www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf
    TODO: add exception to deal with impossible positions
    :param leg: Leg object from which to extract the limb lengths
    :param pos: Cartesian position of the end effector measured in meteres following
                the leg axis. Must be in the form [x, y, z]
    :return angles: returns the angles (in degrees) that correspond to the position
                    in a list [t0, t1, t2]
    """
    X = pos[0]
    Y = pos[1]
    Z = pos[2]

    L1 = leg.L1
    L2 = leg.L2
    L3 = leg.L3

    t1 =  atan2(-Z,Y) - atan2(sqrt(Y**2+Z**2-L1**2),-L1)
    D = (Y**2+Z**2-L1**2+X**2-L2**2-L3**2)/(2*L3*L2)
    # t3_1 = atan2(-sqrt(1 - D**2),D)
    t3 = atan2(sqrt(1 - D**2), D)
    t2 = atan2(X, sqrt(Y**2+Z**2-L1**2)) - atan2(L3*sin(t3),L2+L3*cos(t3))

    return r2d([t1, t2, t3])


def for_k(leg, ang):
    """
    for_k calculates the cartesian position of the end effector based
    on the current leg angles
    :param leg: leg object
    :param ang: leg angles measured in degrees. In a list [t0, t1, t2]
    :return: cartesian position in meters following the leg axis in a
             list [x, y, z]
    """
    L1 = leg.L1
    L2 = leg.L2
    L3 = leg.L3

    ang_r = d2r(ang)

    t1 = ang_r[0]
    t2 = ang_r[1]
    t3 = ang_r[2]

    x = L2*sin(t2) + L3*sin(t2+t3)
    y = -L3*sin(t1)*cos(t2+t3) - L1*cos(t1) - L2*cos(t2)*sin(t1)
    z = L1*sin(t1) - L2*cos(t1)*cos(t2) - L3*cos(t1)*cos(t2+t3)

    return [x, y, z]


def r2d(rads):
    """
    r2d converts radians to degrees
    :param rads: list of radians ([t0, t1, t2]) to convert to degrees
    :return degs: list of resulting degrees [t0, t1, t2]
    """
    degs = []
    degs.append(rads[0]*180/pi)
    degs.append(rads[1]*180/pi)
    degs.append(rads[2]*180/pi)
    return degs


def d2r(degs):
    """
    d2r converts degrees to radians
    :param degs: list of degrees ([t0, t1, t2]) to convert to radians
    :return degs: list of resulting radians [t0, t1, t2]
    """
    rads = []
    rads.append(degs[0]*pi/180)
    rads.append(degs[1]*pi/180)
    rads.append(degs[2]*pi/180)
    return rads


def sign(num):
    """
    sign returns the sign of the input number. TODO: add 0 exception
    :param num: number to extraxt sign off
    :return: sign of number. Either 1 or -1
    """
    return num/abs(num)


def jacobian(leg, angles):
    """
    jacobian calculates the leg jacobian for the current position
    :param leg: leg object
    :param angles: current leg angles in degrees in a list [t0, t1, t2]
    :return jacob: jacobian matrix
    """
    L1 = leg.L1
    L2 = leg.L2
    L3 = leg.L3

    angles = d2r(angles)

    t1 = angles[0]
    t2 = angles[1]
    t3 = angles[2]

    s1 = sin(t1)
    s2 = sin(t2)
    c1 = cos(t1)
    c2 = cos(t2)

    s23 = sin(t2 + t3)
    c23 = cos(t2 + t3)
    
    v12 = L3*c23 + L2*c2
    v13 = L3*c23

    v21 = L1*s1 - L3*c1*c23 - L2*c1*c2
    v22 = L3*s1*s23 + L2*s1*s2
    v23 = L3*s1*s23

    v31 = L1*c1 + L3*s1*c23 + L2*s1*c2
    v32 = L2*c1*s2 + L3*c1*s23
    v33 = L3*c1*s23

    jacob = np.matrix([[0 , v12, v13],[v21, v22, v23],[v31, v32, v33]])
    
    return jacob


def w2v(leg, angles, w):
    """
    w2v converts current leg angular velocities to cartesian velocities
    :param leg: leg object
    :param angles: current angles in degrees in a list [t0, t1, t2]
    :param w: current leg angular velocities in radians per second in a
              list [w0, w1, w2]
    :return vel: cartesian velocities in meters per second. Returned in
                 an np.array in the form of [vx, vy, vz]
    """
    jacob = jacobian(leg, angles)
    vel = np.matmul(jacob, w)
    return vel.A1


def f2t(leg, angles, forces):
    """
    f2t converts current leg force to leg torques
    :param leg: leg object
    :param angles: current angles in degrees in a list [t0, t1, t2]
    :param forces: current leg cartesian forces in N in a list [fx, fy, fz]
    :return torques: corresponding leg torques in Nm. Returned in
                     an np.array in the form of [T0, T1, T2]
    """
    jacob = jacobian(leg, angles)
    torques = np.matmul(jacob.T, forces)
    return torques.A1


def t2f(leg, angles, torques):
    """
    t2f converts current leg torques into leg forces
    :param leg: leg object
    :param angles: current angles in degrees in a list [t0, t1, t2]
    :param torques: current leg torques in Nm in a list [T0, T1, T2]
    :return forces: end effector forces in N in an np.array
                    in the form of [fx, fy, fz]
    """
    jacob = jacobian(leg, angles)
    forces = np.matmul(jacob, torques)
    return forces.A1
