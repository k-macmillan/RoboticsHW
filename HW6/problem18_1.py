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
        self.g = 0.005
        self.start = (0, 5)
        self.goal = (15, 5)
        self.obstacles = []
        self.addObstacles()

    def diffObstacle1X(self, x, y):
        top = -(32 * x - 192)
        bot = (4 * x**2 - 48 * x + y**2 + 156 - 8 * y)**2
        return top / bot

    def diffObstacle2X(self, x, y):
        top = -(162 * x - 1296)
        bot = (9 * x**2 - 144 * x + y**2 + 603 - 12 * y)**2
        return top / bot

    def diffObstacle1Y(self, x, y):
        top = -(8 * y - 32)
        bot = (4 * x**2 - 48 * x + y**2 + 156 - 8 * y)**2
        return top / bot

    def diffObstacle2Y(self, x, y):
        top = -(18 * y - 108)
        bot = (9 * x**2 - 144 * x + y**2 + 603 - 12 * y)**2
        return top / bot

    def diffFieldX(self, x):
        return 2 * (x - 15)

    def diffFieldY(self, y):
        return 2 * (y - 5)

    def addField(self, X, Y):
        return (X - self.goal[0])**2 + ((Y - self.goal[1])**2) - 1

    def addObstacles(self):
        self.obstacles.append((6, 4, 2))
        self.obstacles.append((8, 6, 3))

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
        loops = 1000
        x = [self.start[0]]
        y = [self.start[1]]
        ddx = []
        ddy = []
        for i in range(loops):
            # Calc partial with respect to x
            dx = (self.diffFieldX(x[-1]) +
                  self.diffObstacle1X(x[-1], y[-1]) +
                  self.diffObstacle2X(x[-1], y[-1]))

            # Calc partial with respect to y
            dy = (self.diffFieldY(y[-1]) +
                  self.diffObstacle1Y(x[-1], y[-1]) +
                  self.diffObstacle2Y(x[-1], y[-1]))
            dx *= self.g
            dy *= self.g

            # Made this to check gradients
            ddx.append(dx)
            ddy.append(dy)

            x.append(x[-1] - dx)
            y.append(y[-1] - dy)

        # 'x' because it makes the code cleaner, then assigned to self.x
        self.x = x
        self.y = y
        print('min dx: {}\t max dx: {}'.format(min(ddx), max(ddx)))
        print('min dy: {}\t max dy: {}'.format(min(ddy), max(ddy)))

    def plotObstacles2D(self):
        self.setDXDYDZ()
        ax = plt.gca()
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot(self.x, self.y)
        for x, y, r in self.obstacles:
            ax.add_artist(plt.Circle((x, y), r, color='r'))
        ax.add_artist(plt.Circle((self.start[0], self.start[1]),
                                 radius=0.125, color='g'))
        ax.add_artist(plt.Circle((self.goal[0], self.goal[1]),
                                 radius=0.125, color='g'))
        # ax.plot(myline)
        plt.xlim(-1, 16)
        plt.ylim(-1, 11)
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
