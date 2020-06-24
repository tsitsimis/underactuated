import numpy as np


class SystemLinearizer:
    def __init__(self, system, x0, tau_g_dq, B_dq=None):
        self.system = system
        self.x0 = x0
        self.tau_g_dq = tau_g_dq

        M, M_inv, C, tau_g, B = system.get_manipulator_matrices(x0)
        n_half = M.shape[0]

        if B_dq is None:
            B_dq = np.zeros((n_half, 1))
        self.B_dq = B_dq

        A_lin_top = np.concatenate((
            np.zeros((n_half, n_half)),
            np.eye(n_half)
        ), axis=1)
        A_lin_bottom = np.concatenate((
            M_inv.dot(tau_g_dq),
            -M_inv.dot(C)
        ), axis=1)
        self.A_lin = np.concatenate((A_lin_top, A_lin_bottom), axis=0)

        self.B_lin = np.concatenate((
            np.zeros((n_half, B.shape[1])),
            M_inv.dot(B)
        ), axis=0)
