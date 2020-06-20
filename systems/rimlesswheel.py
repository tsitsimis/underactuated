from systems.pendulum import Pendulum

import time

import numpy as np
import matplotlib.pyplot as plt
import imageio


class RimlessWheel:
    """
    Rimless wheel is the simplest model of a legged robot.
    The system consists of a mass with equally spaced spikes (legs) extending outwards
    and can passively go down a slope given appropriate initial conditions.

    It uses the rigid-body dynamical system of a pendulum for the leg that performs
    the step at each time. The step consists of the pendulum swinging in an upward
    configuration until the next leg touches the ground and the motion continues.

    Parameters
    ----------
    mass : float
        Center of mass

    length : float
        Length of each leg

    alpha : float
        Half of angle (radians) between legs

    gamma : float
        Slope's angle (radians)

    Methods
    -------
    make_step
        Simulates the trajectory of the pendulum (leg)

    walk
        Performs multiple steps and recalculates pendulum's
        initial conditions (angular velocity) based on conservation
        of momentum. Also shows the animation on a matplotlib figure
    """

    def __init__(self, mass, length, gravity, alpha, gamma, x0=None):
        self.mass = mass
        self.length = length
        self.gravity = gravity
        self.alpha = alpha
        self.gamma = gamma

        if gamma > alpha:
            raise ValueError("gamma must be smaller than alpha")

        self.n_legs = int((2 * np.pi) / (2 * alpha))

        self.rot_mat = np.array([
            [np.cos(2*alpha), -np.sin(2*alpha)],
            [np.sin(2*alpha), np.cos(2*alpha)]
        ])

        self.theta_min = gamma - alpha  # angle just after leg starts moving
        self.theta_max = gamma + alpha  # angle just before next leg touches the ground

        if x0 is None:
            theta_0 = self.theta_min + np.pi / 100

            # Two times the minimum velocity to make a step
            omega_0 = np.sqrt(2 * (gravity / length) * (1 - np.cos(gamma - alpha))) * 2

            x0 = np.array([self.transform_theta(theta_0), self.transform_omega(omega_0)])
            self.x0 = x0
        else:
            self.x0 = np.array([self.transform_theta(x0[0]), self.transform_omega(x0[1])])

        self.omega_0 = self.x0[1]

        self.leg = Pendulum(mass=mass, length=length, friction=0, gravity=gravity, x0=x0)

        self.foot_loc = np.array([-1, 1 * np.tan(gamma)])

    @staticmethod
    def transform_theta(theta):
        """
        Transforms wheel angle to pendulum coordinates
        It is also the inverse transform
        """
        return np.pi - theta

    @staticmethod
    def transform_omega(omega):
        """
        Transforms wheel angular velocity to pendulum coordinates
        It is also the inverse transform
        """
        return -omega

    def make_step(self):
        """
        Simulates the trajectory of the pendulum (leg)
        """

        T = 2 * np.pi * np.sqrt(self.length / self.gravity)  # Period of pendulum as upper time bound for leg motion
        dt = self.leg.dt
        time_steps = np.arange(0, T, dt)
        n = time_steps.shape[0]
        states_cache = np.zeros((n, 2))

        i_max = n
        i = 0
        while True:
            states_cache[i, :] = self.leg.x
            i += 1

            self.leg.step()

            # Collision detection
            if self.transform_theta(self.leg.x[0]) >= self.theta_max and self.transform_omega(self.leg.x[1]) > 0:
                i_max = i
                break

        states_cache = states_cache[0:i_max]
        return states_cache

    def walk(self, n_steps, fig, ax, save=False):
        """
        Performs multiple steps and recalculates pendulum's
        initial conditions (angular velocity) based on conservation
        of momentum. Also shows the animation on a matplotlib figure

        Parameters
        ----------
        n_steps : int
            Number of steps to perform
        """

        # Initialize figure
        # Plot's view moves with the wheel to avoid out-of-screen motion
        # Small circles are drawn on the ground (slope) so that the motion is visible

        ax.plot([-1, 100], [np.tan(self.gamma)*1, -np.tan(self.gamma)*100], c="k", linestyle="-")
        [ax.scatter([i], [-np.tan(self.gamma)*i], c="r") for i in range(-1, 100, 1 + int(4*self.length))]

        plt.ion()
        plt.axis("off")
        ax.axis("equal")
        ax.set_xlim(-5 * self.length, 5 * self.length)
        ax.set_ylim(-5 * self.length, 5 * self.length)

        p, = ax.plot([], [], c="k")  # pendulum (leg in motion)
        p_c = ax.scatter([], [], c="k", s=100)  # mass
        p_legs = [ax.plot([], [], c="k")[0] for _ in range(self.n_legs)]  # rest of the legs

        plt.show()

        T = 2 * np.pi * np.sqrt(self.length / self.gravity)
        frames = [None] * n_steps * int(T / self.leg.dt)

        for s in range(n_steps):
            states_cache = self.make_step()

            for i in range(states_cache.shape[0]):
                if i % 10 != 0:
                    continue
                p.set_data([self.foot_loc[0], self.foot_loc[0] + self.length * np.sin(states_cache[i, 0])],
                           [self.foot_loc[1], self.foot_loc[1] - self.length * np.cos(states_cache[i, 0])])

                cm = np.array([self.foot_loc[0] + self.length * np.sin(states_cache[i, 0]),
                               self.foot_loc[1] - self.length * np.cos(states_cache[i, 0])])
                p_c.set_offsets([cm.tolist()])

                for leg_i in range(self.n_legs):
                    direction = np.linalg.matrix_power(self.rot_mat, leg_i).dot(
                        np.array([self.length * np.sin(states_cache[i, 0]), - self.length * np.cos(states_cache[i, 0])])
                    ).flatten()
                    direction /= np.linalg.norm(direction)

                    p_legs[leg_i].set_data([cm[0], cm[0] + self.length*direction[0]],
                                           [cm[1], cm[1] + self.length*direction[1]])

                    ax.set_xlim(cm[0] - 5 * self.length, cm[0] + 5 * self.length)
                    ax.set_ylim(cm[1] - 5 * self.length, cm[1] + 5 * self.length)

                fig.canvas.draw()
                plt.pause(self.leg.dt)

                if save:
                    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                    frames[states_cache.shape[0] * s + i] = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

            # Angular velocity just before the step is complete
            omega_f = np.sqrt(self.omega_0 ** 2 + 4 * (self.gravity / self.length) * np.sin(self.alpha) * np.sin(self.gamma))

            theta_0 = self.theta_min

            # Initial angular velocity of new step based on conservation of momentum
            self.omega_0 = omega_f * np.cos(2 * self.alpha)

            # Initialize pendulum
            self.leg.x = np.array([self.transform_theta(theta_0), self.transform_omega(self.omega_0)])

            # Move pendulum's base in next leg's position on the ground
            self.foot_loc += 2 * self.length * np.sin(self.alpha) * np.array([np.cos(self.gamma), -np.sin(self.gamma)])

        plt.pause(3)

        if save:
            imageio.mimsave(f"./rimlesswheel_{int(time.time())}.gif", [f for f in frames if f is not None], fps=40)
