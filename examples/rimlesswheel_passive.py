from systems.rimlesswheel import RimlessWheel

import matplotlib.pyplot as plt
import numpy as np


wheel = RimlessWheel(mass=1, length=1, gravity=10, alpha=np.pi / 8, gamma=np.pi / 30)

# Animate
fig, ax = plt.subplots(figsize=(5, 5))
wheel.walk(n_steps=15, fig=fig, ax=ax, save=True)
