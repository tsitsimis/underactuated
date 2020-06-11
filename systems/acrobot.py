import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import imageio


class Acrobot:
    """
    Acrobot dynamical system

    Simulates an acrobot based on rigid body manipulator equations.
    The B matrix (mapping from control to angles second derivative)
    is 2x2 meaning that in general case the system can be fully-actuated.
    To model the underactuated acrobot, the first term of control input
    should be zero.
    """

    def __init__(self, m1, m2, l1, l2, gravity, x0, u=lambda t, x: np.array([[0], [0]])):
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.gravity = gravity
        self.u = u

        self.lc1 = l1 / 2
        self.lc2 = l2 / 2
        self.I1 = m1 * l1 ** 2
        self.I2 = m2 * l2 ** 2

        self.x = x0
        self.t = 0
        self.dt = 0.01

    def get_manipulator_matrices(self, x):
        """
        Calculates the manipulator matrices M, C, tau_g and B
        Also returns the inverse of M
        """

        q1, q2, q1_dot, q2_dot = x

        m1, m2, I1, I2, l1, l2, lc1, lc2 = self.m1, self.m2, self.I1, self.I2, self.l1, self.l2, self.lc1, self.lc2
        g = self.gravity

        M = np.array([
            [I1 + I2 + m2 * l1 ** 2 + 2 * m2 * l1 * lc2 * np.cos(q2), I2 + m2 * l1 * lc2 * np.cos(q2)],
            [I2 + m2 * l1 * lc2 * np.cos(q2), I2]
        ])

        M_inv = np.array([
            [M[1, 1], -M[0, 1]],
            [-M[1, 0], M[0, 0]]
        ]) / (M[0, 0]*M[1, 1] - M[0, 1]*M[1, 0])

        C = np.array([
            [-2 * m2 * l1 * lc2 * np.sin(q2) * q2_dot, -m2 * l1 * lc2 * np.sin(q2) * q2_dot],
            [m2 * l1 * lc2 * np.sin(q2) * q1_dot, 0]
        ])

        tau_g = np.array([
            [-m1 * g * lc1 * np.sin(q1) - m2 * g * (l1 * np.sin(q1) + lc2 * np.sin(q1 + q2))],
            [-m2 * g * lc2 * np.sin(q1 + q2)]
        ])

        B = np.eye(2)

        return M, M_inv, C, tau_g, B

    def dynamics(self, t, x):
        """
        Implements the system's differential equations and returns
        state derivatives given current time and state
        """

        q_dot = np.array([[x[2]], [x[3]]])

        M, M_inv, C, tau_g, B = self.get_manipulator_matrices(x)

        x34_dot = M_inv.dot(tau_g + B.dot(self.u(t, x)) - C.dot(q_dot))
        x1_dot = x[2]
        x2_dot = x[3]
        return np.array([x1_dot, x2_dot, x34_dot[0], x34_dot[1]])

    def step(self):
        """
        Applies numerical integration in system dynamics and updates
        current system's state
        """

        sol = solve_ivp(self.dynamics, [self.t, self.t + self.dt], self.x)

        self.x = sol.y[:, -1]
        self.t += self.dt

    def playback(self, fig, ax, T, save=False, show_time=True):
        """
        Simulates the system until time T and animates it in
        a matplotlib figure
        """

        time_steps = np.arange(0, T, self.dt)
        n = time_steps.shape[0]
        states_cache = np.zeros((n, 4))
        for i, t in enumerate(time_steps):
            self.step()
            states_cache[i, :] = self.x

        frames = [None] * n

        plt.ion()
        plt.axis("off")
        ax.axis("equal")
        ax.set_xlim(-1.1 * (self.l1 + self.l2), 1.1 * (self.l1 + self.l2))
        ax.set_ylim(-1.1 * (self.l1 + self.l2), 1.1 * (self.l1 + self.l2))

        ax.scatter([0], [0], marker="o", c="k")
        ax.plot([-self.l1 / 2, self.l1 / 2], [0, 0], c="k", linestyle="--")
        p1, = ax.plot([], [], c="k")
        p2, = ax.plot([], [], c="k")
        middle_joint_p = ax.scatter([], [], c="b", zorder=10)

        if show_time:
            time_text = ax.text(x=0, y=1.2*(self.l1 + self.l2), s="t=0")
        plt.show()

        for i, t in enumerate(time_steps):
            if i % 10 != 0:
                continue
            p1.set_data([0, self.l1 * np.sin(states_cache[i, 0])], [0, -self.l1 * np.cos(states_cache[i, 0])])
            p2.set_data([self.l1 * np.sin(states_cache[i, 0]),
                         self.l1 * np.sin(states_cache[i, 0]) + self.l2 * np.sin(states_cache[i, 0] + states_cache[i, 1])],
                        [-self.l1 * np.cos(states_cache[i, 0]),
                         -self.l1 * np.cos(states_cache[i, 0]) - self.l2 * np.cos(states_cache[i, 0] + states_cache[i, 1])])
            middle_joint_p.set_offsets([[self.l1 * np.sin(states_cache[i, 0]), -self.l1 * np.cos(states_cache[i, 0])]])

            if show_time:
                time_text.set_text(f"t={np.round(t, 1)}")
            fig.canvas.draw()

            plt.pause(self.dt)

            if save:
                image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                frames[i] = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        if save:
            imageio.mimsave(f"./acrobot_{int(time.time())}.gif", [f for f in frames if f is not None], fps=15)
