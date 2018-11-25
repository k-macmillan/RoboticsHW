import numpy as np


# Measurements
m1 = np.array([10.5, 18.2])
m2 = np.array([10.75, 18.0])
m3 = np.array([9.9, 19.1])

# Covariances
V1 = np.array([[0.1, 0.01],
               [0.01, 0.15]])
V2 = np.array([[0.05, 0.005],
               [0.005, 0.05]])
V3 = np.array([[0.2, 0.05],
               [0.05, 0.25]])

# Put data into easy loop format
x_m = np.array([m1[0], m2[0], m3[0]])
y_m = np.array([m1[1], m2[1], m3[1]])
x_var = np.array([V1[0, 0], V2[0, 0], V3[0, 0]])
y_var = np.array([V1[1, 1], V2[1, 1], V3[1, 1]])


# Sensor fusion to obtain x_hat
top_x = 0.0
bot_x = 0.0
for i in range(3):
    top_x += (x_m[i] / x_var[i])
    bot_x += 1 / x_var[i]

# Estimated distance
x_hat = top_x / bot_x

# Sensor fusion to obtain y_hat
top_y = 0.0
bot_y = 0.0
for i in range(3):
    top_y += (y_m[i] / y_var[i])
    bot_y += 1 / y_var[i]

# Estimated distance
y_hat = top_y / bot_y

# Calculate means and reshape for use
x_mean = np.mean(x_m, dtype=np.float64)
y_mean = np.mean(y_m, dtype=np.float64)

# Calculate covariance
cov = 0.0
for i in range(len(x_m)):
    cov += (x_mean - x_m[i]) * (y_mean - y_m[i])

cov /= len(x_m)

print()
print('x_hat: {}'.format(x_hat))
print('y_hat: {}'.format(y_hat))
print('x_var: {}'.format(1 / bot_x))
print('y_var: {}'.format(1 / bot_y))
print('covar: {}'.format(cov))
print()

# Are these the actual values?
# 0.02850471 0.0034289
# 0.0034289  0.03254206
