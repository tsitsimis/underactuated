from systems.acrobot import Acrobot
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt

# Plant
plant = Acrobot(m1=1, m2=2, l1=1, l2=2, gravity=10, x0=np.array([np.pi * 0.8, 0, 0, 0]), underactuated=True)

# Linearize around fixed point (vertical position, zero velocity)
x_goal = np.array([np.pi, 0, 0, 0])
m1, m2, l1, l2, lc1, lc2, g = plant.m1, plant.m2, plant.l1, plant.l2, plant.lc1, plant.lc2, plant.gravity
tau_g_dq = np.array([
    [g * (m1 * lc1 + m2 * l1 + m2 * lc2), m2 * g * lc2],
    [m2 * g * lc2, m2 * g * lc2]
])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=tau_g_dq)

# LQR controller
lqr = LQR(A=sl.A_lin, B=sl.B_lin, Q=np.eye(4), R=np.eye(1))
plant.u = lambda t, x: lqr.controller(t, x - x_goal)

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=10, save=True)
