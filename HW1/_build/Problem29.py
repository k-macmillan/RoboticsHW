import numpy as np
import matplotlib.pyplot as plt
import sys

# constants
D = 10.0
r = 5.0
L = 16.0
circ = 2 * r * np.pi



def xt(w1, w2, t, theta, x0):
    return ((L * (w1 + w2)) / (w1 - w2)) * (np.sin((r / (2.0 * L)) * (w1 - w2) * t + theta) - np.sin(theta)) + x0

def yt(w1, w2, t, theta, y0):
    return (L * (w1 + w2) / (w1 - w2)) * (np.cos((r / 2.0 * L) * (w1 - w2) * t + theta) - np.cos(theta)) + y0

if __name__ == "__main__":
    # Set initial variables
    positions = np.array([[0.0,0.0,0.0],])
    t=5.0
    x_position = 0.0
    y_position = 0.0

    # From time = 0 to 5
    x_position = (2.0 / (2.0 * np.pi) ) * circ * t
    positions = np.concatenate((positions, [[x_position,0.0,0.0],]))


    # From time = 5 to 6
    w1 = 3.0 / (2.0 * np.pi)
    w2 = 4.0 / (2.0 * np.pi)
    t = 1.0
    theta = (r / 2 * L) * (w1 - w2) * t + 0.0
    x_position = xt(w1, w2, t, theta, x_position)
    y_position = yt(w1, w2, t, theta, y_position)
    ary = np.array([[x_position,y_position,0.0],])
    positions = np.concatenate((positions, ary), axis=0)


    # From time = 6 to 10
    w1 = 1.0 / (2.0 * np.pi)
    w2 = 2.0 / (2.0 * np.pi)
    t = 4.0
    theta = (r / 2.0 * L) * (w1 - w2) * t + theta
    x_position = xt(w1, w2, t, theta, x_position)
    y_position = yt(w1, w2, t, theta, y_position)
    positions = np.concatenate((positions, [[x_position,y_position,0.0],]))

    print('Final position: ', ' '.join(map(str, positions[3:4])))
