import pathlib

import copy
import itertools
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
def maps_to_landmarks(map):
    """
    Extract landmarks from a map
    """
    landmarks = np.argwhere(np.array(map) == 1)
    RFID = []
    for i in range(len(landmarks)):
        RFID.append(list(landmarks[i]))
    for j in range(len(RFID)):
        RFID[j].append(0)
    return np.array(RFID)

#Credit to AtsushiSakai for his implementation of GraphSLAM

class Edge:

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0


def cal_observation_sigma():
    sigma = np.zeros((3, 3))
    sigma[0, 0] = C_SIGMA1 ** 2
    sigma[1, 1] = C_SIGMA2 ** 2
    sigma[2, 2] = C_SIGMA3 ** 2

    return sigma


def calc_3d_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = 0

    Rt1 = calc_3d_rotational_matrix(tangle1)
    Rt2 = calc_3d_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma()
    sig2 = cal_observation_sigma()

    edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(x_list, z_list):
    edges = []
    cost = 0.0
    z_ids = list(itertools.combinations(range(len(z_list)), 2))

    for (t1, t2) in z_ids:
        x1, y1, yaw1 = x_list[0, t1], x_list[1, t1], x_list[2, t1]
        x2, y2, yaw2 = x_list[0, t2], x_list[1, t2], x_list[2, t2]

        if z_list[t1] is None or z_list[t2] is None:
            continue  # No observation

        for iz1 in range(len(z_list[t1][:, 0])):
            for iz2 in range(len(z_list[t2][:, 0])):
                if z_list[t1][iz1, 3] == z_list[t2][iz2, 3]:
                    d1 = z_list[t1][iz1, 0]
                    angle1, phi1 = z_list[t1][iz1, 1], z_list[t1][iz1, 2]
                    d2 = z_list[t2][iz2, 0]
                    angle2, phi2 = z_list[t2][iz2, 1], z_list[t2][iz2, 2]

                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, d2, angle2, t1, t2)

                    edges.append(edge)
                    cost += (edge.e.T @ edge.omega @ edge.e)[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


def fill_H_and_b(H, b, edge):
    A, B = calc_jacobian(edge)

    id1 = edge.id1 * STATE_SIZE
    id2 = edge.id2 * STATE_SIZE

    H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T @ edge.omega @ A
    H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T @ edge.omega @ B
    H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T @ edge.omega @ A
    H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T @ edge.omega @ B

    b[id1:id1 + STATE_SIZE] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + STATE_SIZE] += (B.T @ edge.omega @ edge.e)

    return H, b


def graph_based_slam(x_init, hz):
    print("start graph based slam")

    z_list = copy.deepcopy(hz)

    x_opt = copy.deepcopy(x_init)
    nt = x_opt.shape[1]
    n = nt * STATE_SIZE

    for itr in range(MAX_ITR):
        edges = calc_edges(x_opt, z_list)

        H = np.zeros((n, n))
        b = np.zeros((n, 1))

        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        # to fix origin
        H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)

        dx = - np.linalg.inv(H) @ b

        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = dx.T @ dx
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    return x_opt


def calc_input(v, yaw_rate):
    """
    Args:
        v: The agents velocity(meters per second)
        yaw_rate: The rate of change of the agent's heading(radians per second)
    Returns:
        Control input
    """
    return np.array([[v, yaw_rate]]).T


def observation(xTrue, xd, u, RFID, DT):
    """
    Args:
        xTrue: The true pose of the agent
        xd: The estimated pose of the agent
        u: The control input
        RFID: An array of landmarks
        DT: Movements per second
    """
    MAX_RANGE = 30.0
    Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
    R_sim = np.diag([0.1, np.deg2rad(10.0)]) ** 2
    xTrue = motion_model(xTrue, u, DT)

    # add noise to gps x-y
    z = np.zeros((0, 4))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]
        phi = pi_2_pi(math.atan2(dy, dx))
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
            angle_noise = np.random.randn() * Q_sim[1, 1]
            angle += angle_noise
            phi += angle_noise
            zi = np.array([dn, angle, phi, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud, DT)

    return xTrue, z, xd, ud


def motion_model(x, u, DT):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    return x


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

