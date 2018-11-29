from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from simpy import *


class Problem18_1():
    def __init__(self):
        self.x_min = 0
        self.x_max = 15
        self.y_min = 0
        self.y_max = 10
        self.eta = 1
        self.start = (0, 5)
        self.goal = (15, 5)
        self.obstacles = []
        self.addObstacles()

    def diffObstacle1X(self, x, y):
        return -(32 * x - 192) / (4 * x**2 -
                                  48 * x +
                                  y**2 +
                                  156 -
                                  8 * y)**2

    def diffObstacle2X(self, x, y):
        return -(162 * x - 1296) / (9 * x**2 -
                                    144 * x +
                                    y**2 +
                                    603 -
                                    12 * y)**2

    def diffObstacle1Y(self, x, y):
        return -(8 * y - 32) / (4 * x**2 -
                                48 * x +
                                y**2 +
                                156 -
                                8 * y)**2

    def diffObstacle2Y(self, x, y):
        return -(18 * y - 108) / (9 * x**2 -
                                  144 * x +
                                  y**2 +
                                  603 -
                                  12 * y)**2

    def diffFieldX(self, x):
        return 2 * (x - 15)

    def diffFieldY(self, y):
        return 2 * (y - 5)

    def addField(self, X, Y):
        return (X - self.goal[0])**2 + ((Y - self.goal[1])**2) - 1

    def addObstacles(self):
        self.obstacles.append((6, 4, 2, 20))
        self.obstacles.append((8, 6, 3, 20))

    def circle(self, x, y, c):
        """c = (cx, cy, radius, cap)"""
        val = (((x - c[0])**2 / (c[2] * c[2])) +
               ((y - c[1])**2 / (c[2] * c[2])) - 1)
        val[val < 0.001] = 0.001
        val = self.eta / val
        return val

    def setXYZ(self):
        X = np.arange(self.x_min - 1, self.x_max + 1, 0.1)
        Y = np.arange(self.y_min - 1, self.y_max + 1, 0.1)
        X, Y = np.meshgrid(X, Y)
        Z = self.circle(X, Y, self.obstacles[0]) +\
            self.circle(X, Y, self.obstacles[1])
        Z[Z > 150.0] = 150.0
        Z += self.addField(X, Y)
        self.Z = Z.flatten()
        self.X = X.flatten()
        self.Y = Y.flatten()

    def setDXDYDZ(self):
        loops = np.arange(1, 200, 0.5)
        x = []
        y = []
        dist = (self.start[0] - self.goal[0])**2 + (self.start[1] - self.goal[1])**2
        for i in loops:
            x.append(self.diffFieldX(i) + self.diffObstacle1X(i, i) + self.diffObstacle2X(i, i))
            y.append(self.diffFieldY(i) + self.diffObstacle1Y(i, i) + self.diffObstacle2Y(i, i))
        self.UxUy = (x, y)

    def plotObstacles2D(self):
        self.setDXDYDZ()
        exit()
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot_trisurf(self.X, self.Y, self.Z, antialiased=False)
        # ax.plot(myline)
        plt.show()

    def plotObstacles3D(self):
        self.setXYZ()
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot_trisurf(self.X, self.Y, self.Z, antialiased=False)
        # ax.plot(myline)
        plt.show()


if __name__ == '__main__':
    p18 = Problem18_1()
    # p18.plotObstacles3D()
    p18.plotObstacles2D()
