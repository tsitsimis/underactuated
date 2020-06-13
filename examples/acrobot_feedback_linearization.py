from systems.acrobot import Acrobot
from controllers import FLC

import numpy as np
import matplotlib.pyplot as plt

# Plant
plant = Acrobot(m1=1, m2=1, l1=1, l2=1, gravity=10, x0=np.array([np.pi/4, np.pi/4, 0, 0]), underactuated=False)

# Feedback Linearization Controller
# Make system feedback-equivalent to a linear system
# controlled with a simple PD controller
plant.u = FLC(plant, lambda t, x: np.array([[-(x[0] - np.pi) - x[2]], [-(x[1] - 0) - x[3]]])).controller

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=10)
