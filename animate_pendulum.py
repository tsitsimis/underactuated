from systems.pendulum import Pendulum
from controllers import FLC

import numpy as np
import matplotlib.pyplot as plt


# Plant
plant = Pendulum(mass=1, length=1, friction=0.5, gravity=10, x0=[np.pi/4, -5])

# Feedback Linearization Controller
# Make system feedback-equivalent to a linear system
# controlled with a simple PD controller
# plant.u = FLC(plant, lambda t, x: np.array([[-2*x[1] - 2*(x[0] - np.pi)]])).controller

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=50, dt=0.1, save=True, show_time=False)
