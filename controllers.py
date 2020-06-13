import numpy as np
from scipy.linalg import solve_continuous_are


class FLC:
    def __init__(self, plant, v):
        self.plant = plant
        self.v = v

    def controller(self, t, x):
        n = x.shape[0]
        q_dot = x[int(n/2):][:, None]

        M, M_inv, C, tau_g, B = self.plant.get_manipulator_matrices(x)
        u = np.linalg.inv(B).dot(M.dot(self.v(t, x)) + C.dot(q_dot) - tau_g)
        return u


class LQR:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        self.S = solve_continuous_are(A, B, Q, R)

    def controller(self, t, x):
        return -np.linalg.inv(self.R).dot(self.B.T).dot(self.S).dot(x[:, None])
