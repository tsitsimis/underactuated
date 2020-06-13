import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import imageio


class CartPole:
    """
    Cart-Pole dynamical system
    """

    def __init__(self, m_c: float, m_p: float, l: float, gravity: float, x0: np.ndarray, u=None, underactuated=True):
        self.m_c = m_c
        self.m_p = m_p
        self.l = l
        self.gravity = gravity
        self.underactuated = underactuated

        if u is None:
            if underactuated:
                u = lambda t, x: np.array([[0]])
            else:
                u = lambda t, x: np.array([[0], [0]])
        self.u = u

        self.x = x0
        self.t = 0
        self.dt = 0.01

    def get_manipulator_matrices(self, x: np.ndarray):
        """
        Calculates the manipulator matrices M, C, tau_g and B
        Also returns the inverse of M
        """

        d, theta, d_dot, theta_dot = x

        m_c, m_p, l = self.m_c, self.m_p, self.l
        g = self.gravity

        M = np.array([
            [m_c + m_p, m_p * l * np.cos(theta)],
            [m_p * l * np.cos(theta), m_p * l ** 2]
        ])

        M_inv = np.array([
            [M[1, 1], -M[0, 1]],
            [-M[1, 0], M[0, 0]]
        ]) / (M[0, 0] * M[1, 1] - M[0, 1] * M[1, 0])

        C = np.array([
            [0, -m_p * l * theta_dot * np.sin(theta)],
            [0, 0]
        ])

        tau_g = np.array([
            [0],
            [-m_p * g * l * np.sin(theta)]
        ])

        if self.underactuated:
            B = np.array([
                [1],
                [0]
            ])
        else:
            B = np.eye(2)

        return M, M_inv, C, tau_g, B

    def dynamics(self, t: float, x: np.ndarray):
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
        ax.set_xlim(-2 * self.l, 2 * self.l)

        ax.plot([-2 * self.l, 2 * self.l], [0, 0], c="k", linestyle="--")
        p_cart_top, = ax.plot([], [], c="k")
        p_cart_bottom, = ax.plot([], [], c="k")
        p_cart_left_side, = ax.plot([], [], c="k")
        p_cart_right_side, = ax.plot([], [], c="k")
        p_pole, = ax.plot([], [], c="k")
        pole_joint_p = ax.scatter([], [], c="k", zorder=10)

        if show_time:
            time_text = ax.text(x=0, y=2 * self.l, s="t=0")
        plt.show()

        for i, t in enumerate(time_steps):
            if i % 10 != 0:
                continue
            p_cart_top.set_data([states_cache[i, 0] - self.l / 4, states_cache[i, 0] + self.l / 4],
                                [self.l / 5, self.l / 5])
            p_cart_bottom.set_data([states_cache[i, 0] - self.l / 4, states_cache[i, 0] + self.l / 4], [0, 0])
            p_cart_left_side.set_data([states_cache[i, 0] - self.l / 4, states_cache[i, 0] - self.l / 4],
                                      [0, self.l / 5])
            p_cart_right_side.set_data([states_cache[i, 0] + self.l / 4, states_cache[i, 0] + self.l / 4],
                                       [0, self.l / 5])

            p_pole.set_data([states_cache[i, 0], states_cache[i, 0] + self.l * np.sin(states_cache[i, 1])],
                            [self.l / 10, -self.l * np.cos(states_cache[i, 1])])
            pole_joint_p.set_offsets([[states_cache[i, 0], self.l / 10]])

            if show_time:
                time_text.set_text(f"t={np.round(t, 1)}")
            fig.canvas.draw()

            plt.pause(self.dt)

            if save:
                image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                frames[i] = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        if save:
            imageio.mimsave(f"./cartpole_{int(time.time())}.gif", [f for f in frames if f is not None], fps=15)
