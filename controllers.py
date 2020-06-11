import numpy as np


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
