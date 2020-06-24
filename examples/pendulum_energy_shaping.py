from systems.pendulum import Pendulum
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt


# Plant
plant = Pendulum(mass=1, length=1, friction=0.5, gravity=10, x0=np.array([np.pi / 6, 0]))

# Energy shaping controller
k = 100.0
m, g, l = plant.mass, plant.gravity, plant.length
u_max = m*g*l/4


def energy_shaping_controller(t, x, underactuated=True):
    u = -k * x[1] * ((-m*g*l*np.cos(x[0]) + (1/2)*m*(l**2)*(x[1]**2)) - m*g*l)
    if underactuated:
        u = np.clip(u, -u_max, u_max)

    return u


# LQR controller of linearized system
x_goal = np.array([np.pi, 0])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=np.array([[10]]))

lqr = LQR(sl.A_lin, sl.B_lin, 10*np.eye(2), np.eye(1))


def lqr_controller(t, x):
    return lqr.controller(t, x - x_goal)


# Mixed controller
def mixed_controller(t, x):
    if np.linalg.norm(x - x_goal) > 0.1:
        return energy_shaping_controller(t, x)
    return lqr_controller(t, x)


plant.u = mixed_controller

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
controller_text = ax.text(x=-1.0*plant.length, y=1.2 * plant.length, s="", color="r", size=12)


def animation_callback(ax, i, t, states_cache):
    x = states_cache[i, :]
    if np.linalg.norm(x - x_goal) > 0.1:
        controller_text.set_text("Energy Shaping")
    else:
        controller_text.set_text("LQR")


plant.playback(fig=fig, ax=ax, T=15, save=True, callback=animation_callback)

