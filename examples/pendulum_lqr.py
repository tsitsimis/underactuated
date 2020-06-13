from systems.pendulum import Pendulum
from systems.linearization import SystemLinearizer
from controllers import LQR

import numpy as np
import matplotlib.pyplot as plt


# Plant
plant = Pendulum(mass=1, length=1, friction=0.5, gravity=10, x0=np.array([np.pi*3/4, 0]))

x_goal = np.array([np.pi, 0])
sl = SystemLinearizer(plant, x0=x_goal, tau_g_dq=np.array([[10]]))

lqr = LQR(sl.A_lin, sl.B_lin, 10*np.eye(2), np.eye(1))
plant.u = lambda t, x: lqr.controller(t, x - x_goal)

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=4, save=True)

