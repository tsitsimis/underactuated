from systems.cartpole import CartPole
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt


# Plant
plant = CartPole(m_c=1, m_p=1, l=1, gravity=10, x0=np.array([-2, np.pi*3/4, 0, 0]), underactuated=True)

# Linearize around fixed point (vertical position, zero velocity)
x_goal = np.array([0, np.pi, 0, 0])
m_p, l, g = plant.m_p, plant.l, plant.gravity
tau_g_dq = np.array([
    [0, 0],
    [0, m_p * l * g]
])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=tau_g_dq)

# LQR controller
lqr = LQR(A=sl.A_lin, B=sl.B_lin, Q=10*np.eye(4), R=np.eye(1))
plant.u = lambda t, x: lqr.controller(t, x - x_goal)

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=10, save=True)
