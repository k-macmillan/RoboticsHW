import numpy as np


# Time information
dt = 0.1
t = np.arange(0, 2 * np.pi + dt, dt)

# Integrated
y = -np.sin(t) - 0.5 * np.cos(t)
x = 0.5 * y * y

x_dot = y
y_dot = -np.cos(t) + 0.5 * np.sin(t)

# Variance
w = 0.25  # x
