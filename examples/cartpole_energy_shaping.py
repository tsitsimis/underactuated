from systems.cartpole import CartPole
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt

# Plant
plant = CartPole(m_c=1, m_p=1, l=1, gravity=10, x0=np.array([-1, np.pi/6, 0, 0]), underactuated=True)

# Energy shaping controller
m_c, m_p, l, g = plant.m_c, plant.m_p, plant.l, plant.gravity
k_e, k_p, k_d = 5, 10, 10  # gains after experimentation


def energy_shaping_controller(t, x):
    d, theta, d_dot, theta_dot = x

    T = (1/2) * m_p * l**2 * theta_dot**2
    U = -m_p * g * l * np.cos(theta)
    E = T + U
    E_d = m_p * g * l

    u_energy_shaping = k_e * theta_dot * np.cos(theta) * (E - E_d)

    u_desired = u_energy_shaping - k_p * d - k_d * d_dot

    u = (m_c + m_p - m_p * np.cos(theta) ** 2) * u_desired - (
                m_p * g * np.sin(theta) * np.cos(theta) + m_p * l * np.sin(theta) * theta_dot ** 2)

    return u


# LQR controller of linearized system
x_goal = np.array([0, np.pi, 0, 0])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=np.array([[0, 0], [0, m_p*g*l]]))

lqr = LQR(A=sl.A_lin, B=sl.B_lin, Q=10*np.eye(4), R=np.eye(1))


def lqr_controller(t, x):
    return lqr.controller(t, x - x_goal)


# Mixed controller
def mixed_controller(t, x):
    if (abs(x[0]) > 0.1) or (abs(x[1] - np.pi) > np.pi/30):
        return energy_shaping_controller(t, x)
    return lqr_controller(t, x)


plant.u = mixed_controller

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
controller_text = ax.text(x=-2 * plant.l, y=2 * plant.l, s="", color="r", size=12)


def animation_callback(ax, i, t, states_cache):
    x = states_cache[i, :]
    if (abs(x[0]) > 0.1) or (abs(x[1] - np.pi) > np.pi/30):
        controller_text.set_text("Energy Shaping")
    else:
        controller_text.set_text("LQR")


plant.playback(fig=fig, ax=ax, T=12, save=True, callback=animation_callback)
