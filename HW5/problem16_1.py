import numpy as np


measured = np.array([284, 257, 295])
std = np.array([10, 20, 15])

# Calculate variance
var = std * std

# Sensor fusion to obtain x_hat
top = 0.0
bot = 0.0
for i in range(3):
    top += (measured[i] / var[i])
    bot += 1 / var[i]

# Estimated distance
x_hat = top / bot

print('Variance:                     {}'.format(var))
print('Estimated state:              {}\n'.format(x_hat))
