from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np


class Problem18_1():
    def __init__(self):
        self.x_min = 0
        self.x_max = 15
        self.y_min = 0
        self.y_max = 10
        self.start = (0, 5)
        self.goal = (15, 5)
        self.obstacles = []
        self.addObstacles()

    def addObstacles(self):
        self.obstacles.append((6, 4, 2, 60))
        self.obstacles.append((8, 6, 3, 60))

    def circle(self, x, y, c):
        """c = (cx, cy, radius, cap)"""
        val = (((x - c[0]) * (x - c[0]) / (c[2] * c[2])) +
               ((y - c[1]) * (y - c[1]) / (c[2] * c[2])) - 1)
        val[val < 0.05] = 0.05
        val = 1 / val
        return val

    def plotObstacles(self):
        X = np.arange(self.x_min - 1, self.x_max + 1, 0.1)
        Y = np.arange(self.y_min - 1, self.y_max + 1, 0.1)
        X, Y = np.meshgrid(X, Y)
        Z = self.circle(X, Y, self.obstacles[0])
        Z = Z.flatten()
        X = X.flatten()
        Y = Y.flatten()
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_trisurf(X, Y, Z, linewidth=0.5, antialiased=True)
        plt.show()


if __name__ == '__main__':
    p18 = Problem18_1()
    p18.plotObstacles()
