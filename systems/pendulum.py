import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import imageio


class Pendulum:
    """
    Simple pendulum dynamical system

    Simulates a pendulum based on rigid body manipulator equations.
    Accepts an input control function that depends on current time
    and state.
    """

    def __init__(self, mass, length, friction, gravity, x0, u=lambda t, x: 0):
        self.mass = mass
        self.length = length
        self.friction = friction
        self.gravity = gravity
        self.u = u

        self.x = x0
        self.t = 0
        self.dt = 0.01

    def get_manipulator_matrices(self, x):
        """
        Calculates the manipulator matrices M, C, tau_g and B
        Also returns the inverse of M
        """

        q, q_dot = x

        m, l, b, g = self.mass, self.length, self.friction, self.gravity

        M = np.array([[m * l ** 2]])
        M_inv = np.array([[1 / (m * l ** 2)]])

        C = np.array([[b]])

        tau_g = np.array([[- m * g * l * np.sin(q)]])

        B = np.array([[1]])

        return M, M_inv, C, tau_g, B

    def dynamics(self, t, x):
        """
        Implements the system's differential equations and returns
        state derivatives given current time and state
        """

        q_dot = np.array([[x[1]]])

        M, M_inv, C, tau_g, B = self.get_manipulator_matrices(x)
        x2_dot = M_inv.dot(tau_g + B.dot(self.u(t, x)) - C.dot(q_dot))

        return np.array([x[1], x2_dot])

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
        states_cache = np.zeros((n, 2))
        for i, t in enumerate(time_steps):
            self.step()
            states_cache[i, :] = self.x

        frames = [None] * n

        plt.ion()
        plt.axis("off")
        ax.axis("equal")
        ax.set_xlim(-2 * self.length, 2 * self.length)
        ax.set_ylim(-1.1 * self.length, 1.1 * self.length)

        ax.scatter([0], [0], marker="o", c="b")
        ax.plot([-self.length / 2, self.length / 2], [0, 0], c="k", linestyle="--")
        p, = ax.plot([], [], c="k")

        if show_time:
            time_text = ax.text(x=0, y=1.2 * self.length, s="t=0")
        plt.show()

        for i, t in enumerate(time_steps):
            if i % 10 != 0:
                continue
            p.set_data([0, self.length * np.sin(states_cache[i, 0])], [0, -self.length * np.cos(states_cache[i, 0])])
            fig.canvas.draw()

            if show_time:
                time_text.set_text(f"t={np.round(t, 1)}")
            plt.pause(self.dt)

            if save:
                image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                frames[i] = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        if save:
            imageio.mimsave(f"./pendulum_{int(time.time())}.gif", [f for f in frames if f is not None], fps=15)
