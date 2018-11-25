import numpy as np


# Time information
dt = 0.1
t = np.arange(0, 2 * np.pi + dt, dt)  # 64 points

# x_dot = y
# y_dot = -np.cos(t) + 0.5 * np.sin(t)

x = np.zeros(len(t))

# "Physics"
F = np.array([1.0, 0.1],
             [0.1 * np.sin(x), 1.0])

# Variances
V = np.array([0.2, 0.0],
             [0.0, 0.2])
W = 0.25
