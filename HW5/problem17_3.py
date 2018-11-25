import numpy as np


# Time information
dt = 0.1
t = np.arange(0, 2 * np.pi + dt, dt)

# x_dot = y
# y_dot = -np.cos(t) + 0.5 * np.sin(t)

# Variance
w = 0.25  # x
