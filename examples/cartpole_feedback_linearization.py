from systems.cartpole import CartPole
from controllers import FLC

import numpy as np
import matplotlib.pyplot as plt

# Plant
plant = CartPole(m_c=1, m_p=1, l=1, gravity=10, x0=np.array([-1, 0, 0, 0]), underactuated=False)

# Feedback Linearization Controller
# Make system feedback-equivalent to a linear system
# controlled with a simple PD controller
plant.u = FLC(plant, lambda t, x: np.array([[-(x[0] - 0) - x[2]], [-(x[1] - np.pi) - x[3]]])).controller

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
plant.playback(fig=fig, ax=ax, T=10, save=True)
