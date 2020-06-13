from systems.acrobot import Acrobot
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt


# Plant
plant = Acrobot(m1=1, m2=1, l1=1, l2=1, gravity=10, x0=np.array([np.pi*3/4, 0, 0, 0]), underactuated=False)

# Linearize around fixed point (vertical position, zero velocity)
x_goal = np.array([np.pi, 0, 0, 0])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=np.array([[20, 5], [5, 5]]))

# LQR controller
lqr = LQR(A=sl.A_lin, B=sl.B_lin, Q=10*np.eye(4), R=np.eye(2))
plant.u = lambda t, x: lqr.controller(t, x - x_goal)

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=4, save=True)

